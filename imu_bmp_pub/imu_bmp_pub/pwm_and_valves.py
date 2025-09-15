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

        # ---------- Parámetros ----------
        self.declare_parameter('pwm_pins', [12, 13])          # BCM
        self.declare_parameter('pwm_freq', 50.0)              # Hz
        self.declare_parameter('pwm_duty_pct', 4.0)           # % (global inicial)
        self.declare_parameter('valve_pins', [17, 27, 22, 23, 24])

        # Barrido (global; detiene al recibir comandos de duty)
        self.declare_parameter('pwm_sweep_on_start', True)
        self.declare_parameter('pwm_sweep_min_pct', 4.0)
        self.declare_parameter('pwm_sweep_max_pct', 6.0)
        self.declare_parameter('pwm_sweep_step_pct', 0.2)
        self.declare_parameter('pwm_sweep_duration_s', 10.0)

        # Failsafe válvulas
        self.declare_parameter('start_mask', 0)
        self.declare_parameter('auto_off_timeout', 0.0)       # s; 0 = inactivo

        # ---------- Lee parámetros base ----------
        self.pwm_pins = [int(x) for x in self.get_parameter('pwm_pins').value]
        self.pwm_freq = float(self.get_parameter('pwm_freq').value)
        global_duty = float(self.get_parameter('pwm_duty_pct').value)
        self.valve_pins = [int(x) for x in self.get_parameter('valve_pins').value]

        self.sweep_on_start = bool(self.get_parameter('pwm_sweep_on_start').value)
        self.sweep_min = float(self.get_parameter('pwm_sweep_min_pct').value)
        self.sweep_max = float(self.get_parameter('pwm_sweep_max_pct').value)
        self.sweep_step = abs(float(self.get_parameter('pwm_sweep_step_pct').value))
        self.sweep_duration = float(self.get_parameter('pwm_sweep_duration_s').value)

        self.state_mask = int(self.get_parameter('start_mask').value)
        self.auto_off_timeout = float(self.get_parameter('auto_off_timeout').value)

        # ---------- GPIO setup ----------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Mapas por pin
        self._pwm_by_pin = {}     # pin -> GPIO.PWM
        self._duty_by_pin = {}    # pin -> duty (0..100)

        # PWM (software) por pin
        for p in self.pwm_pins:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            pwm = GPIO.PWM(p, self.pwm_freq)
            duty = CLAMP(global_duty, 0.0, 100.0)
            pwm.start(duty)                               # inicia con el duty global
            self._pwm_by_pin[p] = pwm
            self._duty_by_pin[p] = duty
        self.get_logger().info(
            f"PWM(soft) pins={self.pwm_pins} @ {self.pwm_freq:.1f} Hz, duty_init={global_duty:.1f}%")

        # Válvulas LOW
        for pin in self.valve_pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self._apply_mask(self.state_mask)

        # ---------- Subs / Timers ----------
        # Válvulas
        self.sub_mask = self.create_subscription(UInt8, 'valves/bitmask', self._on_mask, 10)
        self._last_cmd_time = self.get_clock().now()
        self.timer_fs = self.create_timer(0.1, self._failsafe_tick) if self.auto_off_timeout > 0.0 else None

        # Duty global por tópico
        self.sub_duty_all = self.create_subscription(Float32, 'pwm/duty_pct', self._on_duty_all_cmd, 10)

        # Duty por pin (un sub por cada pin: /pwm/<pin>/duty_pct)
        self._pin_subs = []
        for p in self.pwm_pins:
            topic = f'pwm/p{p}/duty_pct'
            self._pin_subs.append(
                self.create_subscription(Float32, topic, lambda msg, pin=p: self._on_duty_pin_cmd(pin, msg), 10)
            )

        # Control de barrido global
        self.sub_sweep_start = self.create_subscription(Empty, 'pwm/sweep_start', self._on_sweep_start, 10)
        self.sub_sweep_stop  = self.create_subscription(Empty, 'pwm/sweep_stop',  self._on_sweep_stop, 10)

        # Parámetros dinámicos (global + por pin)
        # Declarar dinámicos por pin tipo pwm_duty_<pin>_pct
        for p in self.pwm_pins:
            self.declare_parameter(f'pwm_duty_{p}_pct', self._duty_by_pin[p])
        self.add_on_set_parameters_callback(self._on_params)

        # Barrido inicial (opcional)
        self._sweep_timer = None
        if self.sweep_on_start:
            self._start_sweep()

    # ---------- Callbacks ----------
    def _on_mask(self, msg: UInt8):
        self.state_mask = int(msg.data) & ((1 << len(self.valve_pins)) - 1)
        self._apply_mask(self.state_mask)
        self._last_cmd_time = self.get_clock().now()
        self.get_logger().info(f"[cmd] bitmask=0b{self.state_mask:0{len(self.valve_pins)}b}")

    def _on_duty_all_cmd(self, msg: Float32):
        self._stop_sweep()
        new_duty = CLAMP(float(msg.data), 0.0, 100.0)
        for p in self.pwm_pins:
            self._apply_pwm_pin(p, new_duty)
            # sincroniza parámetros por pin
            self.set_parameters([rclpy.parameter.Parameter(f'pwm_duty_{p}_pct',
                rclpy.parameter.Parameter.Type.DOUBLE, new_duty)])
        # sincroniza parámetro global
        self.set_parameters([rclpy.parameter.Parameter('pwm_duty_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, new_duty)])
        self.get_logger().info(f"[pwm] duty(all) -> {new_duty:.1f}%")

    def _on_duty_pin_cmd(self, pin: int, msg: Float32):
        if pin not in self._pwm_by_pin:
            self.get_logger().warn(f"[pwm] pin {pin} no es PWM gestionado")
            return
        self._stop_sweep()
        new_duty = CLAMP(float(msg.data), 0.0, 100.0)
        self._apply_pwm_pin(pin, new_duty)
        # sincroniza parámetro por pin
        self.set_parameters([rclpy.parameter.Parameter(f'pwm_duty_{pin}_pct',
            rclpy.parameter.Parameter.Type.DOUBLE, new_duty)])
        self.get_logger().info(f"[pwm] duty(pin {pin}) -> {new_duty:.1f}%")

    def _on_sweep_start(self, _msg: Empty):
        self._start_sweep()

    def _on_sweep_stop(self, _msg: Empty):
        self._stop_sweep()
        self.get_logger().info("[pwm] Barrido detenido")

    def _on_params(self, params):
        ok, reason = True, ''
        for p in params:
            # Global
            if p.name == 'pwm_duty_pct' and p.type_ == rclpy.parameter.Parameter.Type.DOUBLE:
                self._stop_sweep()
                new_duty = CLAMP(p.value, 0.0, 100.0)
                for pin in self.pwm_pins:
                    self._apply_pwm_pin(pin, new_duty)
            elif p.name == 'pwm_freq' and p.type_ == rclpy.parameter.Parameter.Type.DOUBLE:
                freq = max(0.1, float(p.value))
                for pwm in self._pwm_by_pin.values():
                    pwm.ChangeFrequency(freq)
                self.pwm_freq = freq
                self.get_logger().info(f"[pwm] freq -> {self.pwm_freq:.2f} Hz")

            # Por pin: pwm_duty_<pin>_pct
            elif p.name.startswith('pwm_duty_') and p.name.endswith('_pct'):
                mid = p.name[len('pwm_duty_'):-len('_pct')]
                try:
                    pin = int(mid)
                except ValueError:
                    continue
                if pin in self._pwm_by_pin and p.type_ == rclpy.parameter.Parameter.Type.DOUBLE:
                    self._stop_sweep()
                    self._apply_pwm_pin(pin, CLAMP(p.value, 0.0, 100.0))
        return SetParametersResult(successful=ok, reason=reason)

    # ---------- Barrido (global; afecta a todos los PWM) ----------
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
        self.get_logger().info(
            f"[pwm] Barrido iniciado: {lo:.1f}% → {hi:.1f}% en {self.sweep_duration:.2f}s, paso≈{(hi-lo)/steps:.2f}%")

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
        if getattr(self, '_sweep_timer', None) is not None:
            self._sweep_timer.cancel()
            self._sweep_timer = None

    # ---------- Utilidades ----------
    def _apply_pwm_pin(self, pin: int, duty_pct: float):
        duty = CLAMP(float(duty_pct), 0.0, 100.0)
        pwm = self._pwm_by_pin.get(pin)
        if pwm is not None:
            pwm.ChangeDutyCycle(duty)           # se aplica inmediatamente
            self._duty_by_pin[pin] = duty

    def _apply_mask(self, mask: int):
        for i, pin in enumerate(self.valve_pins):
            GPIO.output(pin, GPIO.HIGH if (mask >> i) & 0x1 else GPIO.LOW)

    def _failsafe_tick(self):
        if (self.get_clock().now() - self._last_cmd_time).nanoseconds > int(self.auto_off_timeout * 1e9):
            if self.state_mask != 0:
                self.state_mask = 0
                self._apply_mask(0)
                self.get_logger().warn("Failsafe: sin comandos, todo OFF")

    def destroy_node(self):
        try:
            self._stop_sweep()
            for pwm in self._pwm_by_pin.values():
                try:
                    pwm.ChangeDutyCycle(0.0)
                    pwm.stop()
                except Exception:
                    pass
            for pin in self.valve_pins:
                GPIO.output(pin, GPIO.LOW)
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
