#!/usr/bin/env python3
"""Adaptador TurtleBot 4 (Create 3) -> interfaz esperada por tp1_teach_repeat.

- /tb4_2/wheel_ticks (irobot_create_msgs/WheelTicks) -> /tnr/joint_states (JointState, rad)
- /tb4_2/tf_static -> /tf_static (extrínseca base_link <- láser)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import WheelTicks

TICKS_PER_REV = 508.8


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else '/tb4_2'
    rclpy.init()
    node = Node('tnr_tb4_adapter')
    pub = node.create_publisher(JointState, '/tnr/joint_states', qos_profile_sensor_data)
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    pub_static = node.create_publisher(TFMessage, '/tf_static', static_qos)
    count = {'ticks': 0}

    def on_ticks(msg):
        out = JointState()
        out.header = msg.header
        out.name = ['left_wheel_joint', 'right_wheel_joint']
        k = 2*math.pi/TICKS_PER_REV
        out.position = [msg.ticks_left*k, msg.ticks_right*k]
        pub.publish(out)
        count['ticks'] += 1
        if count['ticks'] == 1:
            node.get_logger().info('Primer WheelTicks recibido: encoders OK')

    def on_static(msg):
        pub_static.publish(msg)
        node.get_logger().info(f'tf_static reenviado: {[t.child_frame_id for t in msg.transforms]}')

    node.create_subscription(WheelTicks, f'{ns}/wheel_ticks', on_ticks, qos_profile_sensor_data)
    node.create_subscription(TFMessage, f'{ns}/tf_static', on_static, static_qos)
    node.get_logger().info(f'Adaptador activo sobre {ns}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
