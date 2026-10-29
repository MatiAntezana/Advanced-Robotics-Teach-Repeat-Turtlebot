"""Teleoperación de terminal con timeout, TwistStamped o Twist."""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, TwistStamped


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('tnr_teleop')
    stamped = node.declare_parameter('publish_stamped', True).value
    speed = node.declare_parameter('speed', 0.30).value
    turn = node.declare_parameter('turn', 1.2).value
    pub = node.create_publisher(TwistStamped if stamped else Twist, '/cmd_vel', 10)

    def send(v=0., w=0.):
        msg = TwistStamped() if stamped else Twist()
        t = msg.twist if stamped else msg
        if stamped:
            msg.header.stamp, msg.header.frame_id = node.get_clock().now().to_msg(), 'base_link'
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(msg)

    settings = termios.tcgetattr(sys.stdin)
    print('w/s: adelante/atrás | a/d: girar | espacio/x: parar | q: salir. Mantener/repetir tecla.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        v, w, last = 0., 0., 0.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in ('q', '\x03'):
                    break
                v, w = {'w': (speed, 0.), 's': (-speed, 0.),
                        'a': (0., turn), 'd': (0., -turn)}.get(key, (0., 0.))
                last = time.monotonic()
            if time.monotonic()-last > .4:
                v, w = 0., 0.
            send(v, w)
    except KeyboardInterrupt:
        pass
    finally:
        send()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
