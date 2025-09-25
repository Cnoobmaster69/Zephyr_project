#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Empty
import RPi.GPIO as GPIO
import math

def clamp(v, lo, hi): return max(lo, min(hi, v))

class DualPwmSweep(Node):
    """
    - PWM solo en BCM 12 y 13 (por defecto)
    - Barrido independiente: p12: 4.0→6.1 %, p13: 4.0→6.3 %
    - Tópicos para cambiar duty xd:
        /pwm/p12/duty_pct   (std_msgs/Float32)
        /pwm/p13/duty_pct   (std_msgs/Float32)
    - Parámetros dinámicos:
        pwm_freq (Hz), p12_* y p13_* para ajustar barridos
    """
    def __init__(self):
        super().__init__('dual_pwm_sweep')

        # -------- Parámetros --------
        self.declare_parameter('pwm_pins', [12, 13])    # BCM
        self.declare_parameter('pwm_freq', 50.0)        # Hz

        # Barrido pin 12
        self.declare_parameter('p12_sweep_on_start', True)
        self.declare_parameter('p12_sweep_min_pct', 4.0)
        self.declare_parameter('p12_sweep_max_pct', 6.1)
        self.declare_parameter('p12_sweep_step_pct', 0.1)
        self.declare_parameter('p12_sweep_duration_s', 5.0)

        # Barrido pin 13
        self.declare_parameter('p13_sweep_on_start', True)
        self.declare_parameter('p13_sweep_min_pct', 4.0)
        self.declare_parameter('p13_sweep_max_pct', 6.3)
        self.declare_parameter('p13_sweep_step_pct', 0.1)
        self.declare_parameter('p13_sweep_duration_s', 5.0)

        # -------- Lee parámetros --------
        self.pwm_pins = [int(x) for x in self.get_parameter('pwm_pins').value]
        assert set(self.pwm_pins) == {12, 13}, "Este nodo espera exactamente los pines BCM 12 y 13"
        self.freq = float(self.get_parameter('pwm_freq').value)

        # Estructuras por pin
        self._pwm = {}           # pin -> GPIO.PWM
        self._duty = {}          # pin -> duty actual (0..100)
        self._sweep = {}         # pin -> dict con 'min','max','step','dur','idx','steps','timer'

        # -------- GPIO --------
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in self.pwm_pins:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
            self._pwm[p] = GPIO.PWM(p, self.freq)
            self._duty[p] = 0.0
            self._pwm[p].start(self._duty[p])

        self.get_logger().info(f"PWM(soft) pins={self.pwm_pins} @ {self.freq:.1f} Hz")

        # -------- Suscriptores (duty por pin) --------
        self.sub12 = self.create_subscription(Float32, 'pwm/p12/duty_pct',
                                              lambda msg: self._on_duty_cmd(12, msg), 10)
        self.sub13 = self.create_subscription(Float32, 'pwm/p13/duty_pct',
                                              lambda msg: self._on_duty_cmd(13, msg), 10)

        # (Opcionales) start/stop global de barridos
        self.sub_sw_start = self.create_subscription(Empty, 'pwm/sweep_start_all',
                                                     self._on_sweep_start_all, 10)
        self.sub_sw_stop  = self.create_subscription(Empty, 'pwm/sweep_stop_all',
                                                     self._on_sweep_stop_all, 10)

        # -------- Parámetros dinámicos --------
        self.add_on_set_parameters_callback(self._on_params)

        # -------- Arranque de barridos --------
        self._setup_sweep_from_params(12, 'p12')
        self._setup_sweep_from_params(13, 'p13')

    # Barridos xd
    def _setup_sweep_from_params(self, pin: int, prefix: str):
        if not bool(self.get_parameter(f'{prefix}_sweep_on_start').value):
            return
        lo = float(self.get_parameter(f'{prefix}_sweep_min_pct').value)
        hi = float(self.get_parameter(f'{prefix}_sweep_max_pct').value)
        step = abs(float(self.get_parameter(f'{prefix}_sweep_step_pct').value))
        dur = float(self.get_parameter(f'{prefix}_sweep_duration_s').value)

        lo = clamp(lo, 0.0, 100.0); hi = clamp(hi, 0.0, 100.0)
        if hi < lo: lo, hi = hi, lo
        steps = max(1, int(math.floor((hi - lo) / max(step, 1e-6))))
        period = dur / max(1, steps)

        # Estado del barrido
        if pin in self._sweep and self._sweep[pin].get('timer'):
            self._sweep[pin]['timer'].cancel()
        self._sweep[pin] = {
            'min': lo, 'max': hi, 'step': step, 'dur': dur,
            'idx': 0, 'steps': steps, 'timer': None, 'period': period
        }
        # Inicializa duty al mínimo y arranca timer
        self._apply_duty(pin, lo)
        self._sweep[pin]['timer'] = self.create_timer(period, lambda p=pin: self._sweep_tick(p))
        self.get_logger().info(
            f"[pwm] pin {pin}: sweep {lo:.2f}% → {hi:.2f}% en {dur:.2f}s, paso≈{(hi-lo)/steps:.3f}%"
        )

    def _sweep_tick(self, pin: int):
        s = self._sweep.get(pin)
        if not s: return
        s['idx'] += 1
        if s['idx'] > s['steps']:
            # Termina barrido y queda en el último valor
            if s['timer']: s['timer'].cancel()
            s['timer'] = None
            self.get_logger().info(f"[pwm] pin {pin}: barrido completado")
            return
        duty = s['min'] + (s['idx'] * (s['max'] - s['min']) / s['steps'])
        self._apply_duty(pin, duty)

    def _stop_sweep(self, pin: int):
        s = self._sweep.get(pin)
        if s and s['timer']:
            s['timer'].cancel()
            s['timer'] = None

    #  Callbacks 
    def _on_duty_cmd(self, pin: int, msg: Float32):
        # Comando manual: detiene barrido de ese pin y aplica duty
        self._stop_sweep(pin)
        self._apply_duty(pin, clamp(float(msg.data), 0.0, 100.0))
        self.get_logger().info(f"[pwm] pin {pin}: duty -> {self._duty[pin]:.2f}% (cmd)")

    def _on_sweep_start_all(self, _msg: Empty):
        self._setup_sweep_from_params(12, 'p12')
        self._setup_sweep_from_params(13, 'p13')
        self.get_logger().info("[pwm] barridos reiniciados (ambos pines)")

    def _on_sweep_stop_all(self, _msg: Empty):
        self._stop_sweep(12)
        self._stop_sweep(13)
        self.get_logger().info("[pwm] barridos detenidos (ambos pines)")

    def _on_params(self, params):
        ok, reason = True, ''
        for p in params:
            if p.name == 'pwm_freq':
                f = max(0.1, float(p.value))
                for pwm in self._pwm.values():
                    pwm.ChangeFrequency(f)
                self.freq = f
                self.get_logger().info(f"[pwm] freq -> {self.freq:.2f} Hz")

        return SetParametersResult(successful=ok, reason=reason)

    #  Utilidades 
    def _apply_duty(self, pin: int, duty_pct: float):
        duty = clamp(duty_pct, 0.0, 100.0)
        self._pwm[pin].ChangeDutyCycle(duty)
        self._duty[pin] = duty

    def destroy_node(self):
        try:
            for pin in list(self._sweep.keys()):
                self._stop_sweep(pin)
            for pwm in self._pwm.values():
                try:
                    pwm.ChangeDutyCycle(0.0); pwm.stop()
                except Exception:
                    pass
            for p in self.pwm_pins:
                try: GPIO.output(p, GPIO.LOW)
                except Exception: pass
            try:
                GPIO.cleanup(self.pwm_pins)
            except Exception:
                GPIO.cleanup()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = DualPwmSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
