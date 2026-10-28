#!/usr/bin/env python3
"""
Genera gráficos o animaciones idénticos al formato marauder_ws/Sprunk:
- Mapa acumulado en gris
- Keyframes y flechas de orientación en negro
- Trayectoria del robot en azul
- Escaneo en vivo en rojo (live scan)
- Setpoint objetivo en círculo verde (target)
- Escaneo almacenado del keyframe objetivo en verde (stored scan)
- Tiempo transcurrido 't = XX.X s'

Uso:
  python3 scripts/plot_run.py --bag evidence/teach_bag_encoders --route routes/lab_route_encoders.json --out evidence/teach_encoders.png
  python3 scripts/plot_run.py --bag evidence/repeat_bag_mixed --route routes/lab_route_mixed.json --out evidence/repeat_mixed.png
  python3 scripts/plot_run.py --bag evidence/repeat_bag_mixed --route routes/lab_route_mixed.json --out evidence/repeat_mixed.mp4 --animate
"""

import argparse
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ROS 2 message deserialization
try:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    import rosbag2_py
    HAS_ROSBAG2 = True
except ImportError:
    HAS_ROSBAG2 = False


def rot2d(pts, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([pts[:, 0] * c - pts[:, 1] * s, pts[:, 0] * s + pts[:, 1] * c])


def transform_points(pts, pose):
    x, y, yaw = pose
    r = rot2d(pts[:, :2], yaw)
    return np.column_stack([r[:, 0] + x, r[:, 1] + y])


def scan_to_points(msg, max_range=8.0):
    ranges = np.array(msg.ranges)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= min(msg.range_max, max_range))
    xs = ranges[valid] * np.cos(angles[valid])
    ys = ranges[valid] * np.sin(angles[valid])
    return np.column_stack([xs, ys])


def read_bag_topics(bag_path):
    """Lee tópicos clave de un rosbag2 (sqlite3 o mcap)."""
    if not HAS_ROSBAG2:
        raise RuntimeError("rosbag2_py no está disponible. Ejecutá con pixi run python...")

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path)), rosbag2_py.ConverterOptions('', ''))

    odoms = []
    scans = []
    setpoints = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        t = stamp_ns * 1e-9

        if topic.endswith('/odom'):
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            odoms.append((t, p.x, p.y, yaw))
        elif topic.endswith('/scan'):
            m = deserialize_message(data, LaserScan)
            scans.append((t, m))
        elif topic.endswith('/setpoint'):
            m = deserialize_message(data, PoseStamped)
            p = m.pose.position
            q = m.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            setpoints.append((t, p.x, p.y, yaw))

    return odoms, scans, setpoints


def load_route_data(route_path):
    with open(route_path, 'r') as f:
        data = json.load(f)
    keyframes = data.get('keyframes', [])
    poses = np.array([kf['pose'] for kf in keyframes]) if keyframes else np.empty((0, 3))
    
    # Acumular mapa de fondo
    map_points = []
    for kf in keyframes:
        pose = kf['pose']
        pts = np.array(kf.get('points', []))
        if len(pts) > 0:
            map_points.append(transform_points(pts, pose))
            
    map_points = np.vstack(map_points) if map_points else np.empty((0, 2))
    return keyframes, poses, map_points


def plot_static(bag_path, route_path, out_file, title=None, target_time=None):
    fig, ax = plt.subplots(figsize=(8, 7), dpi=130)

    # 1. Cargar ruta
    keyframes, poses, map_pts = load_route_data(route_path)

    # 2. Dibujar mapa en gris
    if len(map_pts) > 0:
        # Submuestrear si son demasiados puntos
        step = max(1, len(map_pts) // 15000)
        ax.scatter(map_pts[::step, 0], map_pts[::step, 1], s=1.5, c='0.75', alpha=0.8, rasterized=True)

    # 3. Dibujar keyframes (línea gris, puntos negros, flechas)
    if len(poses) > 0:
        ax.plot(poses[:, 0], poses[:, 1], '-', color='0.35', lw=1.5, zorder=3)
        ax.plot(poses[:, 0], poses[:, 1], 'ko', ms=4, zorder=4)
        ax.quiver(poses[:, 0], poses[:, 1],
                  np.cos(poses[:, 2]), np.sin(poses[:, 2]),
                  scale=22, width=0.003, color='0.25', zorder=4)

    # 4. Cargar datos del bag
    if bag_path and os.path.exists(bag_path):
        odoms, scans, setpoints = read_bag_topics(bag_path)
    else:
        odoms, scans, setpoints = [], [], []

    traj_x, traj_y = [], []
    live_pts = np.empty((0, 2))
    tgt_pose = None
    stored_target_pts = np.empty((0, 2))
    display_t = 0.0

    if odoms:
        t0 = odoms[0][0]
        t_end = odoms[-1][0]
        eval_t = t_end if target_time is None else min(t0 + target_time, t_end)
        display_t = eval_t - t0

        # Trayectoria hasta eval_t
        active_odoms = [o for o in odoms if o[0] <= eval_t]
        traj_x = [o[1] for o in active_odoms]
        traj_y = [o[2] for o in active_odoms]
        current_pose = active_odoms[-1][1:4] if active_odoms else odoms[-1][1:4]

        # Scan más cercano a eval_t
        if scans:
            idx = min(range(len(scans)), key=lambda i: abs(scans[i][0] - eval_t))
            scan_local = scan_to_points(scans[idx][1])
            live_pts = transform_points(scan_local, current_pose)

        # Setpoint objetivo
        if setpoints:
            idx_sp = min(range(len(setpoints)), key=lambda i: abs(setpoints[i][0] - eval_t))
            tgt_pose = setpoints[idx_sp][1:4]
        elif len(poses) > 0:
            tgt_pose = poses[min(len(poses) // 2, len(poses) - 1)]

        if tgt_pose is not None and len(keyframes) > 0:
            # Buscar el keyframe más cercano al target
            dists = [np.hypot(kf['pose'][0] - tgt_pose[0], kf['pose'][1] - tgt_pose[1]) for kf in keyframes]
            best_kf = keyframes[int(np.argmin(dists))]
            pts_kf = np.array(best_kf.get('points', []))
            if len(pts_kf) > 0:
                stored_target_pts = transform_points(pts_kf, best_kf['pose'])

    # 5. Dibujar trayectoria seguida (línea azul)
    if traj_x:
        ax.plot(traj_x, traj_y, '-', color='tab:blue', lw=2.0, zorder=5)

    # 6. Dibujar escaneos y target
    p_live = None
    if len(live_pts) > 0:
        p_live = ax.scatter(live_pts[:, 0], live_pts[:, 1], s=4, c='tab:red', alpha=0.9, zorder=6, label='live scan')

    p_stored = None
    if len(stored_target_pts) > 0:
        p_stored = ax.scatter(stored_target_pts[:, 0], stored_target_pts[:, 1], s=4, c='tab:green', alpha=0.8, zorder=6, label='stored scan of the target')

    p_target = None
    if tgt_pose is not None:
        p_target, = ax.plot([tgt_pose[0]], [tgt_pose[1]], 'o', color='tab:green', ms=9, zorder=7, label='target')

    # 7. Reloj e Información
    bag_name = os.path.basename(os.path.abspath(bag_path)) if bag_path else "route_plot"
    plot_title = title if title else bag_name
    ax.set_title(plot_title, fontsize=12, pad=10)
    ax.text(0.03, 0.95, f't = {display_t:4.1f} s', transform=ax.transAxes,
            fontsize=11, verticalalignment='top', fontweight='bold',
            bbox=dict(boxstyle='square,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'))

    # Leyenda
    handles = []
    if p_live: handles.append(p_live)
    if p_stored: handles.append(p_stored)
    if p_target: handles.append(p_target)
    if handles:
        ax.legend(handles=handles, loc='lower right', framealpha=0.85, fontsize=9)

    ax.set_aspect('equal')
    ax.grid(True, linestyle='-', alpha=0.3)
    ax.set_xlabel('x (m)', fontsize=10)
    ax.set_ylabel('y (m)', fontsize=10)

    # Ajustar límites automáticamente
    all_x = list(poses[:, 0]) if len(poses) else []
    all_y = list(poses[:, 1]) if len(poses) else []
    if len(map_pts) > 0:
        step = max(1, len(map_pts) // 1000)
        all_x.extend(map_pts[::step, 0])
        all_y.extend(map_pts[::step, 1])
    if traj_x:
        all_x.extend(traj_x)
        all_y.extend(traj_y)

    if all_x and all_y:
        x_min, x_max = np.percentile(all_x, 1) - 0.6, np.percentile(all_x, 99) + 0.6
        y_min, y_max = np.percentile(all_y, 1) - 0.6, np.percentile(all_y, 99) + 0.6
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_file)), exist_ok=True)
    fig.savefig(out_file, dpi=130)
    plt.close(fig)
    print(f"✅ Gráfico generado exitosamente en: {out_file}")


def main():
    parser = argparse.ArgumentParser(description="Genera gráficos estilo marauder_ws/Sprunk para Teach & Repeat")
    parser.add_argument('--bag', type=str, default='', help='Ruta a la carpeta del rosbag')
    parser.add_argument('--route', type=str, required=True, help='Ruta al archivo lab_route_*.json')
    parser.add_argument('--out', type=str, default='evidence/run_plot.png', help='Archivo de salida (.png)')
    parser.add_argument('--title', type=str, default='', help='Título del gráfico')
    parser.add_argument('--time', type=float, default=None, help='Segundo específico para graficar el scan en vivo')
    args = parser.parse_args()

    plot_static(args.bag, args.route, args.out, title=args.title, target_time=args.time)


if __name__ == '__main__':
    main()
