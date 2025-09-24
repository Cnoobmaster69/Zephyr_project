#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import UInt8, Float32, Empty
import RPi.GPIO as GPIO
import math

CLAMP = lambda v, lo, hi: max(lo, min(hi, v))

class PwmAndValves(Node):
    def __init__(self):
        super().__init__('pwm_and_valves')

        # ----------- Parámetros (PWM principal) -----------
        self.declare_parameter('pwm_pins', [12, 13])     # BCM
        self.declare_parameter('pwm_freq', 50.0)         # Hz
        self.declare_parameter('pwm_duty_pct', 7.0)      # %
        # Barrido global para pwm_pins
        self.declare_parameter('pwm_sweep_on_start', True)
        self.declare_parameter('pwm_sweep_min_pct', 4.0)
        self.declare_parameter('pwm_sweep_max_pct', 6.1)
        self.declare_parameter('pwm_sweep_step_pct', 0.2)
        self.declare_parameter('pwm_sweep_duration_s', 10.0)

        # ----------- Parámetros (válvulas con PWM) -----------
        self.declare_parameter('valve_pins', [17, 27, 22, 23, 24])   # BCM
        self.declare_parameter('valve_pwm_freq', 8.0)               # Hz
        self.declare_parameter('valve_duty_pct', 100.0)               # % (global inicial)
        # Failsafe + bitmask inicial
        self.declare_parameter('start_mask', 0)
        self.declare_parameter('auto_off_timeout', 0.0)              # s; 0 = desactivado

        # ----------- Lee parámetros -----------
        self.pwm_pins = [int(x) for x in self.get_parameter('pwm_pins').value]
        self.pwm_freq = float(self.get_parameter('pwm_freq').value)
        self.pwm_duty_global = float(self.get_parameter('pwm_duty_pct').value)
        self.sweep_on_start = bool(self.get_parameter('pwm_sweep_on_start').value)
        self.sweep_min = float(self.get_parameter('pwm_sweep_min_pct').value)
        self.sweep_max = float(self.get_parameter('pwm_sweep_max_pct').value)
        self.sweep_step = abs(float(self.get_parameter('pwm_sweep_step_pct').value))
        self.sweep_duration = float(self.get_parameter('pwm_sweep_duration_s').value)

        self.valve_pins = [int(x) for x in self.get_parameter('valve_pins').value]
        self.valve_pwm_freq = float(self.get_parameter('valve_pwm_freq').value)
        self.valve_duty_global = float(self.get_parameter('valve_duty_pct').value)

        self.state_mask = int(self.get_parameter('start_mask').value)
        self.auto_off_timeout = float(self.get_parameter('auto_off_timeout').value)

        # ----------- GPIO setup -----------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # PWM principales (software PWM)
        self._pwm_by_pin = {}
        self._duty_by_pin = {}
        for p in self.pwm_pins:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            pwm = GPIO.PWM(p, self.pwm_freq)
            duty = CLAMP(self.pwm_duty_global, 0.0, 100.0)
            pwm.start(duty)
            self._pwm_by_pin[p] = pwm
            self._duty_by_pin[p] = duty
        self.get_logger().info(f"PWM(soft) pins={self.pwm_pins} @ {self.pwm_freq:.1f} Hz, duty_init={self.pwm_duty_global:.1f}%")

        # PWM para válvulas: arrancan en 0% (apagadas) y se habilitan con el bitmask
        self._valve_pwm_by_pin = {}
        self._valve_duty_by_pin = {}
        for vp in self.valve_pins:
            GPIO.setup(vp, GPIO.OUT, initial=GPIO.LOW)
            vpwm = GPIO.PWM(vp, self.valve_pwm_freq)
            vpwm.start(0.0)  # off (no toggle)
            self._valve_pwm_by_pin[vp] = vpwm
            self._valve_duty_by_pin[vp] = CLAMP(self.valve_duty_global, 0.0, 100.0)

        # Aplica bitmask inicial (0 = todas 0% duty)
        self._apply_valve_mask(self.state_mask)

        # ----------- Subs / Timers -----------
        # Bitmask de válvulas (enable/disable PWM por válvula)
        self.sub_mask = self.create_subscription(UInt8, 'valves/bitmask', self._on_mask, 10)
        self._last_cmd_time = self.get_clock().now()
        self.timer_fs = self.create_timer(0.1, self._failsafe_tick) if self.auto_off_timeout > 0.0 else None

        # Duty global y por pin para PWM principales
        self.sub_pwm_all = self.create_subscription(Float32, 'pwm/duty_pct', self._on_duty_all_cmd, 10)
        self._pin_subs = []
        for p in self.pwm_pins:
            topic = f'pwm/p{p}/duty_pct'
            self._pin_subs.append(self.create_subscription(Float32, topic, lambda msg, pin=p: self._on_duty_pin_cmd(pin, msg), 10))

        # Duty global y por válvula (PWM de válvulas)
        self.sub_valve_all = self.create_subscription(Float32, 'valves/pwm/duty_pct', self._on_valve_duty_all_cmd, 10)
        self._valve_pin_subs = []
        for vp in self.valve_pins:
            vtopic = f'valves/pwm/p{vp}/duty_pct'
            self._valve_pin_subs.append(self.create_subscription(Float32, vtopic, lambda msg, pin=vp: self._on_valve_duty_pin_cmd(pin, msg), 10))

        # Barrido para PWM principales
        self.sub_sweep_start = self.create_subscription(Empty, 'pwm/sweep_start', self._on_sweep_start, 10)
        self.sub_sweep_stop  = self.create_subscription(Empty, 'pwm/sweep_stop',  self._on_sweep_stop, 10)
        self._sweep_timer = None
        if self.sweep_on_start:
            self._start_sweep()

        # Parámetros dinámicos
        for p in self.pwm_pins:
            self.declare_parameter(f'pwm_duty_{p}_pct', self._duty_by_pin[p])
        for vp in self.valve_pins:
            self.declare_parameter(f'valve_duty_{vp}_pct', self._valve_duty_by_pin[vp])
        self.add_on_set_parameters_callback(self._on_params)

    # ---------- Callbacks de control ----------
    def _on_mask(self, msg: UInt8):
        self.state_mask = int(msg.data) & ((1 << len(self.valve_pins)) - 1)
        self._apply_valve_mask(self.state_mask)
        self._last_cmd_time = self.get_clock().now()
        self.get_logger().info(f"[valves] mask=0b{self.state_mask:0{len(self.valve_pins)}b}")

    # PWM principales (global)
    def _on_duty_all_cmd(self, msg: Float32):
        self._stop_sweep()
        duty = CLAMP(float(msg.data), 0.0, 100.0)
        for pin in self.pwm_pins:
            self._apply_pwm_pin(pin, duty)
            self.set_parameters([rclpy.parameter.Parameter(f'pwm_duty_{pin}_pct',
                rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.set_parameters([rclpy.parameter.Parameter('pwm_duty_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.get_logger().info(f"[pwm] duty(all) -> {duty:.1f}%")

    # PWM principales (por pin)
    def _on_duty_pin_cmd(self, pin: int, msg: Float32):
        self._stop_sweep()
        duty = CLAMP(float(msg.data), 0.0, 100.0)
        self._apply_pwm_pin(pin, duty)
        self.set_parameters([rclpy.parameter.Parameter(f'pwm_duty_{pin}_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.get_logger().info(f"[pwm] duty(pin {pin}) -> {duty:.1f}%")

    # PWM válvulas (global)
    def _on_valve_duty_all_cmd(self, msg: Float32):
        duty = CLAMP(float(msg.data), 0.0, 100.0)
        for vp in self.valve_pins:
            self._valve_duty_by_pin[vp] = duty
            # Aplica solo a las válvulas habilitadas por mask
            if self._is_valve_enabled(vp):
                self._valve_pwm_by_pin[vp].ChangeDutyCycle(duty)
            self.set_parameters([rclpy.parameter.Parameter(f'valve_duty_{vp}_pct',
                rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.set_parameters([rclpy.parameter.Parameter('valve_duty_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.get_logger().info(f"[valves] duty(all) -> {duty:.1f}%")

    # PWM válvulas (por válvula/pin)
    def _on_valve_duty_pin_cmd(self, pin: int, msg: Float32):
        if pin not in self._valve_pwm_by_pin:
            self.get_logger().warn(f"[valves] pin {pin} no registrado como válvula")
            return
        duty = CLAMP(float(msg.data), 0.0, 100.0)
        self._valve_duty_by_pin[pin] = duty
        if self._is_valve_enabled(pin):
            self._valve_pwm_by_pin[pin].ChangeDutyCycle(duty)
        self.set_parameters([rclpy.parameter.Parameter(f'valve_duty_{pin}_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, duty)])
        self.get_logger().info(f"[valves] duty(pin {pin}) -> {duty:.1f}%")

    # ---------- Parámetros dinámicos ----------
    def _on_params(self, params):
        ok, reason = True, ''
        for p in params:
            # PWM principales
            if p.name == 'pwm_duty_pct':
                self._stop_sweep()
                duty = CLAMP(float(p.value), 0.0, 100.0)
                for pin in self.pwm_pins:
                    self._apply_pwm_pin(pin, duty)
            elif p.name == 'pwm_freq':
                freq = max(0.1, float(p.value))
                for pwm in self._pwm_by_pin.values():
                    pwm.ChangeFrequency(freq)   # API RPi.GPIO
                self.pwm_freq = freq
                self.get_logger().info(f"[pwm] freq -> {self.pwm_freq:.2f} Hz")
            elif p.name.startswith('pwm_duty_') and p.name.endswith('_pct'):
                mid = p.name[len('pwm_duty_'):-len('_pct')]
                try:
                    pin = int(mid)
                except ValueError:
                    continue
                if pin in self._pwm_by_pin:
                    self._stop_sweep()
                    self._apply_pwm_pin(pin, CLAMP(float(p.value), 0.0, 100.0))

            # PWM válvulas
            elif p.name == 'valve_duty_pct':
                duty = CLAMP(float(p.value), 0.0, 100.0)
                for vp in self.valve_pins:
                    self._valve_duty_by_pin[vp] = duty
                    if self._is_valve_enabled(vp):
                        self._valve_pwm_by_pin[vp].ChangeDutyCycle(duty)
            elif p.name == 'valve_pwm_freq':
                vfreq = max(0.1, float(p.value))
                for vpwm in self._valve_pwm_by_pin.values():
                    vpwm.ChangeFrequency(vfreq)
                self.valve_pwm_freq = vfreq
                self.get_logger().info(f"[valves] freq -> {self.valve_pwm_freq:.2f} Hz")
            elif p.name.startswith('valve_duty_') and p.name.endswith('_pct'):
                mid = p.name[len('valve_duty_'):-len('_pct')]
                try:
                    vp = int(mid)
                except ValueError:
                    continue
                if vp in self._valve_pwm_by_pin:
                    duty = CLAMP(float(p.value), 0.0, 100.0)
                    self._valve_duty_by_pin[vp] = duty
                    if self._is_valve_enabled(vp):
                        self._valve_pwm_by_pin[vp].ChangeDutyCycle(duty)
        return SetParametersResult(successful=ok, reason=reason)

    # ---------- Barrido (solo PWM principales) ----------
    def _start_sweep(self):
        self._stop_sweep()
        lo = CLAMP(self.sweep_min, 0.0, 100.0)
        hi = CLAMP(self.sweep_max, 0.0, 100.0)
        if hi < lo:
            lo, hi = hi, lo
        steps = max(1, int(math.floor((hi - lo) / max(0.001, self.sweep_step))))
        period = self.sweep_duration / max(1, steps)
        self._sweep_idx = 0
        self._sweep_lo = lo
        self._sweep_hi = hi
        self._sweep_steps = steps
        self._sweep_timer = self.create_timer(period, self._sweep_tick)
        for pin in self.pwm_pins:
            self._apply_pwm_pin(pin, lo)
        self.get_logger().info(f"[pwm] Barrido: {lo:.1f}%→{hi:.1f}% en {self.sweep_duration:.2f}s, paso≈{(hi-lo)/steps:.2f}%")

    def _sweep_tick(self):
        self._sweep_idx += 1
        if self._sweep_idx > self._sweep_steps:
            self._stop_sweep()
            self.get_logger().info("[pwm] Barrido completado; modo manual")
            return
        duty = self._sweep_lo + (self._sweep_idx * (self._sweep_hi - self._sweep_lo) / self._sweep_steps)
        for pin in self.pwm_pins:
            self._apply_pwm_pin(pin, duty)

    def _stop_sweep(self):
        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self._sweep_timer = None

    # ---------- Callbacks de barrido ----------
    def _on_sweep_start(self, _msg: Empty):
    # Relanza el barrido global de los 2 PWMs principales
        self._start_sweep()
        self.get_logger().info("[pwm] Barrido (sweep) iniciado por tópico")

    def _on_sweep_stop(self, _msg: Empty):
    # Detiene el barrido global de los 2 PWMs principales
        self._stop_sweep()
        self.get_logger().info("[pwm] Barrido (sweep) detenido por tópico")


    # ---------- Utilidades ----------
    def _apply_pwm_pin(self, pin: int, duty_pct: float):
        duty = CLAMP(float(duty_pct), 0.0, 100.0)
        if pin in self._pwm_by_pin:
            self._pwm_by_pin[pin].ChangeDutyCycle(duty)  # aplica inmediato
            self._duty_by_pin[pin] = duty

    def _is_valve_enabled(self, pin: int) -> bool:
        # mapea bit i ↔ self.valve_pins[i]
        try:
            idx = self.valve_pins.index(pin)
        except ValueError:
            return False
        return ((self.state_mask >> idx) & 0x1) == 1

    def _apply_valve_mask(self, mask: int):
        mask = mask & ((1 << len(self.valve_pins)) - 1)
        for i, vp in enumerate(self.valve_pins):
            enabled = (mask >> i) & 0x1
            if enabled:
                # habilitado: duty propio
                self._valve_pwm_by_pin[vp].ChangeDutyCycle(self._valve_duty_by_pin[vp])
            else:
                # deshabilitado: 0% (sin toggle)
                self._valve_pwm_by_pin[vp].ChangeDutyCycle(0.0)

    def _failsafe_tick(self):
        if self.auto_off_timeout <= 0.0:
            return
        if (self.get_clock().now() - self._last_cmd_time).nanoseconds > int(self.auto_off_timeout * 1e9):
            if self.state_mask != 0:
                self.state_mask = 0
                self._apply_valve_mask(0)
                self.get_logger().warn("Failsafe: sin comandos, todas las válvulas OFF")

    def destroy_node(self):
        try:
            self._stop_sweep()
            for pwm in self._pwm_by_pin.values():
                try:
                    pwm.ChangeDutyCycle(0.0); pwm.stop()
                except Exception:
                    pass
            for vpwm in self._valve_pwm_by_pin.values():
                try:
                    vpwm.ChangeDutyCycle(0.0); vpwm.stop()
                except Exception:
                    pass
            # además aseguramos GPIO en LOW
            for pin in self.pwm_pins + self.valve_pins:
                try: GPIO.output(pin, GPIO.LOW)
                except Exception: pass
            try:
                GPIO.cleanup(self.pwm_pins + self.valve_pins)
            except Exception:
                GPIO.cleanup()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = PwmAndValves()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

