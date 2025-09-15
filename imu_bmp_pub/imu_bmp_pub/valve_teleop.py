#!/usr/bin/env python3
import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

HELP = """
Flechas:   ↑ frente (17&22)   ↓ atrás (27&23)   ← giro izq (17&23)   → giro der (27&22)
Espacio:   todo OFF
WASD:      atajos (W=↑, S=OFF, A=←, D=→)
Q:         salir
"""

# Orden de pines en pwm_and_valves: [17, 27, 22, 23, 24]
# Bit positions:   0    1    2    3    4
MASK_OFF   = 0b00000
MASK_FRONT = 0b00101  # 17 & 22
MASK_BACK  = 0b01010  # 27 & 23
MASK_LEFT  = 0b01001  # 17 & 23
MASK_RIGHT = 0b00110  # 27 & 22

class ValveTeleop(Node):
    def __init__(self):
        super().__init__('valve_teleop')
        self.pub = self.create_publisher(UInt8, 'valves/bitmask', 10)
        self.get_logger().info("Teleop de válvulas listo. " + HELP.replace('\n', ' '))

    def publish_mask(self, mask: int):
        msg = UInt8()
        msg.data = mask & 0xFF
        self.pub.publish(msg)
        self.get_logger().info(f"cmd bitmask=0b{mask:05b}")

def get_key(timeout=0.1):
    """Lee una tecla (incluye flechas '\x1b[A/B/C/D') sin bloquear."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            # posible secuencia de flecha
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and select.select([sys.stdin], [], [], 0.01)[0]:
                    ch3 = sys.stdin.read(1)
                    return '\x1b[' + ch3
            return ch1
        return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = ValveTeleop()
    try:
        last = None
        node.publish_mask(MASK_OFF)
        while rclpy.ok():
            key = get_key(0.1)
            if key is None:
                continue
            k = key.lower()
            if key == '\x1b[A' or k == 'w':       # up
                last = MASK_FRONT
            elif key == '\x1b[B'or k == 's':                 # down
                last = MASK_BACK
            elif key == '\x1b[D' or k == 'a':     # left
                last = MASK_LEFT
            elif key == '\x1b[C' or k == 'd':     # right
                last = MASK_RIGHT
            elif k == ' ':            # stop/off
                last = MASK_OFF
            elif k == 'q':
                node.publish_mask(MASK_OFF)
                break
            else:
                continue

            node.publish_mask(last)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_mask(MASK_OFF)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
