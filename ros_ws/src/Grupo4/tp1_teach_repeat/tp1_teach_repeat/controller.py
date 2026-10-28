"""Control polar relativo; detención por pérdida de setpoints o de reloj."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from .core import polar_command
from .localization import ns, yaw
from .runtime import spin_until_stopped


class Controller(Node):
    def __init__(self, **kwargs):
        super().__init__('tnr_controller', **kwargs)
        defaults = dict(publish_stamped=True, control_rate=20., target_timeout=0.6,
                        position_tolerance=0.035, yaw_tolerance=0.06,
                        max_v=0.18, max_w=1.0, k_rho=0.9,
                        k_alpha=2.5, k_beta=-0.6, k_yaw=1.5)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not (self.p['k_rho'] > 0 and self.p['k_beta'] < 0 and self.p['k_alpha'] > self.p['k_rho']):
            raise ValueError('Se requiere k_rho>0, k_beta<0, k_alpha>k_rho')
        for k in ('control_rate', 'target_timeout', 'max_v', 'max_w',
                  'position_tolerance', 'yaw_tolerance', 'k_yaw'):
            if self.p[k] <= 0:
                raise ValueError(f'{k} debe ser positivo')
        self.stamped = self.p.pop('publish_stamped')
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
        self.target, self.received = None, 0.
        self.create_subscription(PoseStamped, '/tnr/setpoint', self.on_target, 10)
        # Timer de pared: sigue enviando cero aunque se pause /clock.
        self.create_timer(1/self.p['control_rate'], self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_target(self, msg):
        p = msg.pose.position
        if msg.header.frame_id != 'base_link' or not all(math.isfinite(v) for v in (p.x, p.y, yaw(msg.pose.orientation))):
            self.target = None
            return
        self.target, self.received = msg, time.monotonic()

    def command(self, v, w):
        msg = TwistStamped() if self.stamped else Twist()
        twist = msg.twist if self.stamped else msg
        if self.stamped:
            msg.header.frame_id, msg.header.stamp = 'base_link', self.get_clock().now().to_msg()
        twist.linear.x, twist.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def tick(self):
        v, w = 0., 0.
        if self.target is not None:
            is_sim = bool(self.get_parameter('use_sim_time').value)
            fresh = (time.monotonic() - self.received <= self.p['target_timeout'])
            if is_sim:
                age = (self.get_clock().now().nanoseconds - ns(self.target.header.stamp))*1e-9
                fresh = fresh and (0 <= age <= self.p['target_timeout'])
            if fresh:
                p = self.target.pose.position
                options = {k: val for k, val in self.p.items() if k not in ('control_rate', 'target_timeout')}
                v, w = polar_command((p.x, p.y, yaw(self.target.pose.orientation)), **options)
                self.get_logger().info(f'Tracking target: rho={math.hypot(p.x, p.y):.2f}m -> v={v:.2f}m/s, w={w:.2f}rad/s', throttle_duration_sec=2.0)
            else:
                self.get_logger().warning('Setpoint expirado o reloj desfasado', throttle_duration_sec=2.0)
        else:
            self.get_logger().warning('Esperando setpoint en /tnr/setpoint...', throttle_duration_sec=3.0)
        self.command(v, w)


def main(args=None):
    # Mantener el contexto válido hasta publicar cero al recibir Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Controller()
    try:
        spin_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.command(0., 0.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
