"""Único sistema de localización para Teach y Repeat.

Mixto: T_odom_base(t) = T_odom_wheel * T_wheel_base(t). Cada ICP
actualiza T_odom_wheel al tiempo del scan, conservando el avance posterior.
"""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from visualization_msgs.msg import MarkerArray

from .core import (ICP, between, compose, inverse, interpolate, load_route,
                   save_route, transform_points, wheel_step, wrap, alignment_quality)
from . import visualization
from .runtime import spin_until_stopped
from .mapping import reference_grid


def ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def fill_pose(msg, p):
    msg.position.x, msg.position.y = float(p[0]), float(p[1])
    msg.orientation.z, msg.orientation.w = math.sin(p[2]/2), math.cos(p[2]/2)
    return msg


def cloud_message(points, stamp):
    pts = np.asarray(points, dtype='<f4').reshape(-1, 3)
    cloud = PointCloud2()
    cloud.header.frame_id, cloud.header.stamp = 'odom', stamp
    cloud.height, cloud.width, cloud.is_dense = 1, len(pts), True
    cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    cloud.point_step, cloud.row_step, cloud.data = 12, 12*len(pts), pts.tobytes()
    return cloud


class Localization(Node):
    def __init__(self, **kwargs):
        super().__init__('tnr_localization', **kwargs)
        defaults = dict(stage='teach', mode='mixed', route_file='routes/route.json',
                        overwrite=False, wheel_radius=0.033, wheel_separation=0.160,
                        left_joint='wheel_left_joint', right_joint='wheel_right_joint',
                        joint_angles_wrapped=False, keyframe_distance=0.20,
                        keyframe_angle=0.25, position_tolerance=0.035,
                        yaw_tolerance=0.06, icp_distance=0.25, icp_fitness=0.55,
                        icp_rmse=0.10, min_range=0.12, max_range=5.0,
                        initial_x=0.0, initial_y=0.0, initial_yaw=0.0,
                        scan_timeout=0.8, teach_submap_keyframes=3, publish_rate=50.0,
                        map_resolution=.04, keyframe_label_stride=3)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.mode, self.stage = self.p['mode'], self.p['stage']
        if self.mode not in ('encoders', 'lidar', 'mixed') or self.stage not in ('teach', 'repeat'):
            raise ValueError('stage: teach/repeat; mode: encoders/lidar/mixed')
        for k in ('wheel_radius', 'wheel_separation', 'keyframe_distance',
                  'keyframe_angle', 'position_tolerance', 'yaw_tolerance', 'scan_timeout', 'publish_rate', 'map_resolution'):
            if self.p[k] <= 0:
                raise ValueError(f'{k} debe ser positivo')
        self.route_file = Path(self.p['route_file']).expanduser().absolute()
        if self.stage == 'teach' and self.route_file.exists() and not self.p['overwrite']:
            raise FileExistsError(f'{self.route_file} existe; usar otra ruta u overwrite:=true')
        self.frames, self.trajectory = [], []
        if self.stage == 'repeat':
            data = load_route(self.route_file, self.mode != 'encoders')
            self.frames = data['keyframes']
            if len(self.frames) < 2:
                raise ValueError(
                    f'La ruta {self.route_file} tiene un solo keyframe. '
                    'En Teach, mover el robot al menos 0.20 m o girarlo 0.25 rad '
                    'antes de guardar y ejecutar Repeat.'
                )
            self.teach_trajectory = data.get('trajectory', [])
        self.index, self.done = 0, False
        self.alignment = None
        self.last_navigation_ns = -1
        self.last_visual_state = None
        self.map_cache_key, self.map_cache = None, None
        self.pose = np.array([self.p['initial_x'], self.p['initial_y'], self.p['initial_yaw']])
        self.wheel = np.zeros(3)
        self.correction = self.pose.copy()
        self.history = deque(maxlen=2000)
        self.last_joint = None
        self.last_scan_ns = None
        self.last_cloud, self.last_scan_pose, self.raw_scan = None, None, None
        self.prev_cloud, self.prev_scan_pose = None, None
        self.pending_scan = None
        self.last_publish_ns = -1
        self.last_path_publish_ns = -1
        self.last_stamp = None
        self.measurement_stamp = None
        self.started_ns = None
        self.icp = None if self.mode == 'encoders' else ICP(
            self.p['icp_distance'], self.p['icp_fitness'], self.p['icp_rmse'])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, '/tnr/odom', 10)
        self.pub_rel = self.create_publisher(Odometry, '/rel_odom', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/tnr/setpoint', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_done = self.create_publisher(Bool, '/tnr/done', latched)
        self.pub_status = self.create_publisher(String, '/tnr/status', latched)
        self.pub_path = self.create_publisher(RosPath, f'/tnr/{self.stage}_path', latched)
        self.pub_teach = self.create_publisher(RosPath, '/tnr/teach_path', latched) if self.stage == 'repeat' else None
        self.pub_markers = self.create_publisher(MarkerArray, '/tnr/keyframes', latched)
        self.pub_navigation = self.create_publisher(MarkerArray, '/tnr/navigation_markers', latched)
        self.pub_alignment = self.create_publisher(String, '/tnr/alignment', latched)
        self.pub_map = self.create_publisher(PointCloud2, '/tnr/reference_map', latched)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/tnr/reference_grid', latched)
        self.pub_walls = self.create_publisher(MarkerArray, '/tnr/reference_walls', latched)
        self.pub_scan = self.create_publisher(PointCloud2, '/tnr/localized_scan', qos_profile_sensor_data)
        self.path = RosPath()
        if self.mode != 'lidar':
            self.create_subscription(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/tnr/save_route', self.on_save)
        self.create_timer(1.0, self.publish_visuals)
        self.get_logger().info(f'{self.stage.upper()} modo={self.mode}, ruta={self.route_file}')

    def on_joint(self, msg):
        t = ns(msg.header.stamp)
        left_name, right_name = self.p['left_joint'], self.p['right_joint']
        if left_name not in msg.name or right_name not in msg.name:
            candidates_left = [n for n in msg.name if 'left' in n.lower() and 'wheel' in n.lower()]
            candidates_right = [n for n in msg.name if 'right' in n.lower() and 'wheel' in n.lower()]
            if candidates_left and candidates_right:
                left_name, right_name = candidates_left[0], candidates_right[0]
        try:
            values = np.array([msg.position[msg.name.index(k)]
                               for k in (left_name, right_name)])
        except (ValueError, IndexError):
            self.get_logger().warning('JointState sin ambas ruedas por nombre', throttle_duration_sec=3.)
            return
        if not np.isfinite(values).all():
            return
        if self.last_joint is not None and t <= self.last_joint[0]:
            if t < self.last_joint[0]:
                self.get_logger().error('Tiempo de encoders retrocedió: reiniciar Teach/Repeat')
            return
        velocity = (0., 0.)
        if self.last_joint is not None:
            delta = values - self.last_joint[1]
            if self.p['joint_angles_wrapped']:
                delta = np.array([wrap(d) for d in delta])
            step = wheel_step(*delta, self.p['wheel_radius'], self.p['wheel_separation'])
            self.wheel = compose(self.wheel, step)
            dt = (t-self.last_joint[0])*1e-9
            velocity = (self.p['wheel_radius']*sum(delta)/2/dt, step[2]/dt)
        self.last_joint = (t, values)
        self.measurement_stamp = msg.header.stamp
        self.history.append((t, self.wheel.copy()))
        if self.started_ns is None:
            self.started_ns = t
        self.pose = compose(self.correction, self.wheel)
        # Un scan puede llegar antes del encoder que permite interpolarlo.
        if self.pending_scan is not None and ns(self.pending_scan.header.stamp) <= t:
            scan, self.pending_scan = self.pending_scan, None
            self.on_scan(scan)
            self.pose = compose(self.correction, self.wheel)
        if self.stage == 'teach' and (t-self.started_ns)*1e-9 > self.p['scan_timeout']:
            if self.last_scan_ns is None or (t-self.last_scan_ns)*1e-9 > self.p['scan_timeout']:
                self.record(self.pose, msg.header.stamp)
        self.publish(self.pose, msg.header.stamp, velocity)

    def cloud(self, msg):
        # Usar la extrínseca real base_link <- láser; nunca suponer sensor centrado.
        tf = None
        for base in ('base_link', 'tb4_2/base_link', 'tb4_1/base_link'):
            try:
                tf = self.buffer.lookup_transform(base, msg.header.frame_id, Time())
                break
            except TransformException:
                pass
        if tf is None:
            self.get_logger().warning(f'Esperando TF base_link <- {msg.header.frame_id}', throttle_duration_sec=3.)
            return None
        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(len(r))*msg.angle_increment
        valid = np.isfinite(r) & (r >= max(msg.range_min, self.p['min_range'])) & (r <= min(msg.range_max, self.p['max_range']))
        r, a = r[valid], a[valid]
        points = np.column_stack((r*np.cos(a), r*np.sin(a), np.zeros(len(r))))
        tr = tf.transform.translation
        return transform_points(points, (tr.x, tr.y, yaw(tf.transform.rotation)))

    def on_scan(self, msg):
        t = ns(msg.header.stamp)
        if self.last_scan_ns is not None and t <= self.last_scan_ns:
            return
        wheel_at_scan = None
        if self.mode != 'lidar':
            wheel_at_scan = interpolate(self.history, t)
            if wheel_at_scan is None:
                if not self.history or t > self.history[-1][0]:
                    self.pending_scan = msg
                return
        points = self.cloud(msg)
        if points is None or len(points) < 20:
            return
        estimate = self.pose.copy() if self.mode == 'lidar' else compose(self.correction, wheel_at_scan)
        if self.icp is not None:
            if self.prev_cloud is not None:
                guess = between(self.prev_scan_pose, estimate) if self.mode == 'mixed' else np.zeros(3)
                step = self.icp.align(points, self.prev_cloud, guess)
                dt = (t-self.last_scan_ns)*1e-9
                if step is None or np.linalg.norm(step[:2]) > max(0.15, 0.5*dt) or abs(step[2]) > max(0.25, 2.*dt):
                    self.get_logger().warning('ICP rechazado: calidad o movimiento no plausible', throttle_duration_sec=2.)
                    return
                estimate = compose(self.prev_scan_pose, step)
            if self.stage == 'teach' and self.p['teach_submap_keyframes'] > 0:
                # Las anclas fijas evitan sumar el error de cada scan consecutivo.
                nearby = [f for f in self.frames[-int(self.p['teach_submap_keyframes']):] if len(f['points']) >= 20]
                if nearby:
                    submap = np.concatenate([transform_points(f['points'], f['pose']) for f in nearby])
                    matched = self.icp.align(points, submap, estimate)
                    if matched is not None:
                        correction = between(estimate, matched)
                        if np.linalg.norm(correction[:2]) < .20 and abs(correction[2]) < .15:
                            estimate = matched
            if self.stage == 'repeat':
                active = self.frames[self.index]
                guess = between(active['pose'], estimate)
                matched = self.icp.align(points, active['points'], guess)
                if matched is None:
                    if self.mode == 'lidar':
                        self.get_logger().warning('Sin alineamiento al keyframe Teach', throttle_duration_sec=2.)
                        return
                else:
                    correction = between(guess, matched)
                    if np.linalg.norm(correction[:2]) < 0.35 and abs(correction[2]) < 0.6:
                        estimate = compose(active['pose'], matched)
            self.prev_cloud, self.prev_scan_pose = points, estimate.copy()
            if self.mode == 'mixed':
                self.correction = compose(estimate, inverse(wheel_at_scan))
        self.last_scan_ns = t
        self.last_cloud, self.last_scan_pose = points, estimate.copy()
        # Resultado real de la localización al tiempo del scan. En mixto, la TF
        # histórica puede contener aún la predicción anterior a recibir este ICP.
        # Publicar la nube en odom evita dibujar esa predicción como pose corregida.
        self.pub_scan.publish(cloud_message(transform_points(points, estimate), msg.header.stamp))
        self.raw_scan = dict(stamp_ns=t, frame=msg.header.frame_id,
                             angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                             range_min=msg.range_min, range_max=msg.range_max,
                             ranges=[float(r) if math.isfinite(r) else None for r in msg.ranges])
        # Medir el escaneo real respecto al ancla; no desplazarlo solo para dibujarlo.
        anchors = self.frames[-int(max(1, self.p['teach_submap_keyframes'])):] if self.stage == 'teach' else [self.frames[self.index]]
        targets = [transform_points(f['points'], f['pose']) for f in anchors if len(f['points']) >= 20]
        if targets:
            self.alignment = dict(scan_stamp_ns=t, source='teach_submap' if self.stage == 'teach' else 'repeat_keyframe',
                                  **alignment_quality(points, np.concatenate(targets), estimate, self.p['icp_distance']))
            self.pub_alignment.publish(String(data=json.dumps(self.alignment)))
        if self.stage == 'teach':
            # DDS puede entregar encoders antes de la extrínseca estática del láser.
            # Completar anclas que nacieron sin scan, expresando esta observación
            # real en su marco; raw_scan conserva su timestamp original.
            filled = False
            for frame in self.frames:
                if not frame['points']:
                    frame['points'] = transform_points(points, between(frame['pose'], estimate)).tolist()
                    frame['scan'] = self.raw_scan
                    filled = True
            if filled:
                self.persist()
            self.record(estimate, msg.header.stamp, points, self.raw_scan)
        if self.mode == 'lidar':
            old = self.pose.copy()
            dt = (t-self.last_publish_ns)*1e-9 if self.last_publish_ns >= 0 else 0.
            delta = between(old, estimate)
            self.pose = estimate
            self.measurement_stamp = msg.header.stamp
            self.publish(estimate, msg.header.stamp,
                         (delta[0]/dt, delta[2]/dt) if dt > 0 else (0., 0.))
        # En mixto publica el próximo encoder, con su timestamp real y corrección aplicada.

    def record(self, pose, stamp, points=None, raw=None, force=False):
        if self.frames:
            d = between(self.frames[-1]['pose'], pose)
            if not force and np.linalg.norm(d[:2]) < self.p['keyframe_distance'] and abs(d[2]) < self.p['keyframe_angle']:
                return
            if force and np.linalg.norm(d[:2]) < 0.005 and abs(d[2]) < 0.01:
                return
        if points is None and self.last_cloud is not None:
            # Lleva la última nube al marco exacto de este keyframe.
            points = transform_points(self.last_cloud, between(pose, self.last_scan_pose))
            raw = self.raw_scan
        previous = self.frames[-1]['pose'] if self.frames else pose
        self.frames.append(dict(pose=np.asarray(pose).tolist(), stamp_ns=ns(stamp),
                                relative_pose=between(previous, pose).tolist(),
                                points=[] if points is None else np.asarray(points).tolist(), scan=raw))
        self.persist()
        if rclpy.ok():
            self.get_logger().info(f'Keyframe {len(self.frames)-1}: {np.round(pose, 3).tolist()}')
        self.publish_visuals()

    def persist(self):
        if self.stage == 'teach' and self.frames:
            save_route(self.route_file, self.frames, self.mode, self.trajectory)

    def on_save(self, request, response):
        if self.stage != 'teach' or self.last_stamp is None:
            response.success, response.message = False, 'Teach sin mediciones, o etapa Repeat'
        else:
            self.record(self.pose, self.measurement_stamp, force=True)
            self.persist()
            response.success, response.message = True, f'{len(self.frames)} keyframes en {self.route_file}'
        return response

    def tf(self, parent, child, pose, stamp):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id, msg.header.stamp = parent, child, stamp
        msg.transform.translation.x, msg.transform.translation.y = float(pose[0]), float(pose[1])
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        return msg

    def publish(self, pose, stamp, velocity):
        t = ns(stamp)
        if t <= self.last_publish_ns:
            return
        if self.last_publish_ns >= 0 and t-self.last_publish_ns < 1e9/self.p['publish_rate']:
            return
        self.last_publish_ns, self.last_stamp = t, stamp
        if self.stage == 'repeat':
            target_index = min(self.index + 1, len(self.frames)-1)
            error = between(pose, self.frames[target_index]['pose'])
            is_final = (target_index == len(self.frames)-1)
            pos_tol = self.p['position_tolerance'] if is_final else max(self.p['position_tolerance']*2.0, 0.08)
            yaw_tol = self.p['yaw_tolerance'] if is_final else math.pi
            if np.linalg.norm(error[:2]) <= pos_tol and abs(error[2]) <= yaw_tol:
                if is_final:
                    if not self.done:
                        self.get_logger().info('REPEAT COMPLETADO')
                    self.index = target_index
                    self.done = True
                elif not self.done:
                    self.index = target_index
                    self.get_logger().info(f'Referencia activa {self.index}; destino {self.index+1}')
                    target_index += 1
                    error = between(pose, self.frames[target_index]['pose'])
            reference = np.array(self.frames[self.index]['pose'])
            target = np.array(self.frames[target_index]['pose'])
        else:
            reference = np.array(self.frames[-1]['pose']) if self.frames else np.zeros(3)
        relative = between(reference, pose)
        transforms = [self.tf('odom', 'reference', reference, stamp),
                      self.tf('reference', 'base_link', relative, stamp)]
        if self.stage == 'repeat':
            transforms += [self.tf('base_link', 'map_current_keyframe', inverse(relative), stamp),
                           self.tf('map_current_keyframe', 'map_next_keyframe', between(reference, target), stamp),
                           self.tf('base_link', 'setpoint', error, stamp)]
            goal = PoseStamped()
            goal.header.frame_id, goal.header.stamp = 'base_link', stamp
            fill_pose(goal.pose, np.zeros(3) if self.done else error)
            self.pub_target.publish(goal)
        self.broadcaster.sendTransform(transforms)
        for pub, frame, p in ((self.pub_odom, 'odom', pose), (self.pub_rel, 'reference', relative)):
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp, odom.child_frame_id = frame, stamp, 'base_link'
            fill_pose(odom.pose.pose, p)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = map(float, velocity)
            # Incertidumbre nominal; no representa una estimación probabilística.
            for i, value in ((0, .01), (7, .01), (14, 1e6), (21, 1e6), (28, 1e6), (35, .02)):
                odom.pose.covariance[i] = value
            pub.publish(odom)
        self.pub_done.publish(Bool(data=self.done))
        self.pub_status.publish(String(data=f'{self.stage}:{self.mode}:keyframe={self.index}:done={self.done}'))
        state = (len(self.frames), self.index, self.done)
        if state != self.last_visual_state:
            self.last_visual_state = state
            self.publish_visuals()
        if t-self.last_navigation_ns >= 100_000_000:
            self.last_navigation_ns = t
            self.publish_navigation(pose, stamp)
        if not self.trajectory or np.linalg.norm(between(self.trajectory[-1][1:], pose)) > .01:
            self.trajectory.append([t, *pose.tolist()])
            p = PoseStamped()
            p.header.frame_id, p.header.stamp = 'odom', stamp
            fill_pose(p.pose, pose)
            self.path.poses.append(p)
            self.path.poses = self.path.poses[-10000:]
        self.path.header.frame_id, self.path.header.stamp = 'odom', stamp
        if t-self.last_path_publish_ns >= 100_000_000:
            self.last_path_publish_ns = t
            self.pub_path.publish(self.path)

    def publish_navigation(self, pose, stamp):
        quality = None if self.alignment is None else dict(self.alignment)
        if quality is not None and self.get_clock().now().nanoseconds-quality['scan_stamp_ns'] > self.p['scan_timeout']*1e9:
            quality['stale'] = True
        target_index = min(self.index+1, len(self.frames)-1) if self.frames else 0
        target_pose = self.frames[target_index]['pose'] if self.stage == 'repeat' else None
        positions = np.asarray([f['pose'] for f in self.frames]) if self.frames else np.asarray([pose])
        mode_position = ((positions[:, 0].min()+positions[:, 0].max())/2,
                         positions[:, 1].max()+.75, 0.)
        self.pub_navigation.publish(visualization.navigation(pose, target_pose, target_index,
                                                             self.done, stamp, quality,
                                                             mode=self.mode, stage=self.stage,
                                                             mode_position=mode_position))

    def publish_visuals(self):
        if self.last_stamp is None or not self.frames or not rclpy.ok():
            return
        stamp = self.last_stamp
        self.publish_navigation(self.pose, stamp)
        all_points = []
        for f in self.frames:
            if len(f['points']):
                all_points.append(transform_points(f['points'], f['pose']))
        self.pub_markers.publish(visualization.keyframes(self.frames, self.stage, self.index, self.done, stamp,
                                                       int(self.p['keyframe_label_stride'])))
        if all_points:
            self.pub_map.publish(cloud_message(np.concatenate(all_points), stamp))
            cache_key = (len(self.frames), sum(len(f['points']) for f in self.frames))
            if cache_key != self.map_cache_key:
                self.map_cache = reference_grid(self.frames, self.p['map_resolution'])
                self.map_cache_key = cache_key
            if self.map_cache is not None:
                grid, origin = self.map_cache
                message = OccupancyGrid()
                message.header.frame_id, message.header.stamp = 'odom', stamp
                message.info.resolution = float(self.p['map_resolution'])
                message.info.height, message.info.width = map(int, grid.shape)
                message.info.origin.position.x, message.info.origin.position.y = map(float, origin)
                message.info.origin.orientation.w = 1.
                message.data = grid.ravel().tolist()
                self.pub_grid.publish(message)
                self.pub_walls.publish(visualization.walls(grid, origin, self.p['map_resolution'], stamp))
        if self.pub_teach:
            path = RosPath()
            path.header.frame_id, path.header.stamp = 'odom', stamp
            for f in self.teach_trajectory or [[0, *k['pose']] for k in self.frames]:
                p = PoseStamped()
                p.header = path.header
                fill_pose(p.pose, f[1:])
                path.poses.append(p)
            self.pub_teach.publish(path)

    def finish(self):
        if self.stage == 'teach' and self.last_stamp is not None:
            self.record(self.pose, self.measurement_stamp, force=True)
            self.persist()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Localization()
        spin_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
