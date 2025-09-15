import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO  # provisto por rpi-lgpio en Bookworm

class Blink(Node):
    def __init__(self):
        super().__init__('blink_gpio')
        # Parámetros
        self.declare_parameter('pin', 4)        # BCM 4  (pin físico 7)
        self.declare_parameter('period', 0.2)   # segundos
        self.declare_parameter('chip', 'gpiochip4')  # compatibilidad; sin uso aquí

        self.pin = int(self.get_parameter('pin').value)
        self.period = float(self.get_parameter('period').value)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self._state = False

        # Temporizador ROS 2: alterna cada "period"
        self.timer = self.create_timer(self.period, self._tick)
        self.get_logger().info(f'Parpadeando BCM{self.pin} cada {self.period}s')

    def _tick(self):
        self._state = not self._state
        GPIO.output(self.pin, GPIO.HIGH if self._state else GPIO.LOW)

    def destroy_node(self):
        try:
            GPIO.output(self.pin, GPIO.LOW)
            GPIO.cleanup(self.pin)
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = Blink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
