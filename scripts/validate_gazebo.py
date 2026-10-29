#!/usr/bin/env python3
"""Ejecuta una prueba automatizada de Teach y Repeat en Gazebo."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tp1_teach_repeat.core import between
from tp1_teach_repeat.localization import yaw


def command(args):
    """Ejecuta un comando del sistema y captura su salida."""
    print('$ ' + shlex.join(args), flush=True)
    return subprocess.run(args, check=True, text=True, capture_output=True, timeout=20)


def start(stage, mode, route):
    """Lanza el archivo launch correspondiente en un proceso independiente."""
    args = ['ros2', 'launch', 'tp1_teach_repeat', stage+'.launch.py',
            'mode:='+mode, 'route_file:='+str(route), 'overwrite:=true']
    print('$ ' + shlex.join(args), flush=True)
    return subprocess.Popen(args, start_new_session=True)


def stop(process):
    """Detiene limpiamente un proceso mediante SIGINT y SIGTERM."""
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


class Probe(Node):
    """Nodo para monitorear topicos, transformadas y comandar teleoperacion."""

    def __init__(self):
        super().__init__('tp1_gazebo_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.latest, self.done = None, False
        self.last_command = None
        self.scans, self.joints, self.tf_count = 0, 0, 0
        self.tf_parents, self.tf_times, self.tf_regressions = {}, {}, []
        self.samples = []
        self.capture = False
        self.create_subscription(Odometry, '/tnr/odom', self.odom, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/tnr/done', self.completed, qos)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)
        self.create_subscription(TwistStamped, '/cmd_vel', self.velocity, 10)
        self.save_client = self.create_client(Trigger, '/tnr/save_route')

    def odom(self, msg):
        self.latest = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                       yaw(msg.pose.pose.orientation)]
        if self.capture:
            self.samples.append([self.get_clock().now().nanoseconds, *self.latest])

    def scan(self, msg):
        self.scans += 1

    def joint(self, msg):
        self.joints += 1

    def completed(self, msg):
        self.done = msg.data

    def velocity(self, msg):
        self.last_command = (msg.twist.linear.x, msg.twist.angular.z)

    def transforms(self, msg):
        for t in msg.transforms:
            child = t.child_frame_id
            self.tf_parents.setdefault(child, set()).add(t.header.frame_id)
            stamp = t.header.stamp.sec*10**9 + t.header.stamp.nanosec
            if stamp < self.tf_times.get(child, 0):
                self.tf_regressions.append(child)
            self.tf_times[child] = stamp
            self.tf_count += 1

    def pump(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def wait(self, predicate, timeout, label):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('Timeout: '+label)
            rclpy.spin_once(self, timeout_sec=.05)

    def send(self, v=0., w=0.):
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def drive(self, seconds, v=0., w=0.):
        begin = self.get_clock().now().nanoseconds
        wall_end = time.monotonic()+max(30, seconds*10)
        while (self.get_clock().now().nanoseconds-begin)*1e-9 < seconds:
            if time.monotonic() > wall_end:
                raise RuntimeError('El reloj de Gazebo no avanza')
            self.send(v, w)
            self.pump(.05)
        self.send()

    def save(self):
        self.wait(self.save_client.service_is_ready, 15, 'servicio de guardado')
        future = self.save_client.call_async(Trigger.Request())
        self.wait(future.done, 10, 'respuesta de guardado')
        result = future.result()
        assert result.success, result.message
        print(result.message, flush=True)

    def reset_robot(self):
        self.send()
        result = command(['gz', 'service', '-s', '/world/default/set_pose',
                          '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                          '--timeout', '5000', '--req',
                          'name: "burger" position: {x: -2.0 y: -0.5 z: 0.01} orientation: {w: 1.0}'])
        assert 'true' in result.stdout, result.stdout + result.stderr
        self.drive(1.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--modes', nargs='+', default=['encoders', 'lidar', 'mixed'])
    parser.add_argument('--output', default='validation/gazebo')
    args = parser.parse_args()
    output = Path(args.output).absolute()
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Probe()
    process = None
    results = []
    try:
        node.wait(lambda: node.scans >= 3 and node.joints >= 3, 45, 'sensores de Gazebo')
        print(f'Sensores: {node.scans} scans, {node.joints} joint_states', flush=True)
        for mode in args.modes:
            print(f'\nValidando {mode}...', flush=True)
            node.reset_robot()
            route = output / f'{mode}_route.json'
            previous_mtime = route.stat().st_mtime_ns if route.exists() else None
            node.latest, node.done, node.samples = None, False, []
            process = start('teach', mode, route)
            node.wait(lambda: node.latest is not None and route.exists()
                      and route.stat().st_mtime_ns != previous_mtime, 40, 'Teach listo')
            node.capture = True
            node.drive(3., v=.12)
            node.drive(1.)
            node.drive(2., w=.35)
            node.drive(1.)
            node.drive(2.5, v=.12)
            node.drive(1.)
            node.save()
            node.capture = False
            teach = node.samples.copy()
            stop(process)
            process = None
            data = json.loads(route.read_text())
            assert len(data['keyframes']) >= 4
            if mode != 'encoders':
                assert all(len(k['points']) >= 20 for k in data['keyframes'])
            node.reset_robot()
            node.latest, node.done, node.samples = None, False, []
            node.tf_parents, node.tf_times, node.tf_regressions = {}, {}, []
            node.capture = True
            process = start('repeat', mode, route)
            node.wait(lambda: node.done, 120, 'Repeat completado')
            node.wait(lambda: node.last_command == (0., 0.), 3, 'velocidad cero al finalizar')
            node.capture = False
            repeat = node.samples.copy()
            final = between(node.latest, data['keyframes'][-1]['pose'])
            assert np.linalg.norm(final[:2]) < .05 and abs(final[2]) < .08, final
            expected = {'reference': 'odom', 'base_link': 'reference',
                        'map_current_keyframe': 'base_link',
                        'map_next_keyframe': 'map_current_keyframe', 'setpoint': 'base_link'}
            for child, parent in expected.items():
                assert node.tf_parents.get(child) == {parent}, (child, node.tf_parents)
            assert not node.tf_regressions, node.tf_regressions
            result = dict(mode=mode, keyframes=len(data['keyframes']), completed=True,
                          final_position_error_m=float(np.linalg.norm(final[:2])),
                          final_yaw_error_rad=float(abs(final[2])),
                          stopped_at_end=True,
                          tf_parents={k: sorted(v) for k, v in node.tf_parents.items()},
                          tf_regressions=node.tf_regressions)
            results.append(result)
            (output/f'{mode}_trajectories.json').write_text(json.dumps(dict(teach=teach, repeat=repeat)))
            (output/'results.json').write_text(json.dumps(results, indent=2))
            print('Resultado: '+json.dumps(result), flush=True)
            stop(process)
            process = None
            node.drive(.5)
    finally:
        stop(process)
        node.send()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
