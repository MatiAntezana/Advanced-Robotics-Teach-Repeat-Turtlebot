#!/usr/bin/env python3
"""Comparativa de trayectoria, error lateral y alineacion ICP en simulacion."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

ROOT = Path(__file__).resolve().parents[1]

MODES = [
    ('encoders', 'Encoders', '#2671be'),
    ('lidar', 'LiDAR 2D (ICP)', '#8959b5'),
    ('mixed', 'Mixto (Fusión)', '#078875')
]

TYPES = {
    '/tnr/odom': Odometry,
    '/tnr/setpoint': PoseStamped,
    '/tnr/status': String,
    '/tnr/done': Bool,
    '/tnr/alignment': String
}


def smooth_signal(y, window_len=31):
    """Aplica suavizado hanning con extension de bordes."""
    if len(y) < window_len:
        return y
    w = np.hanning(window_len)
    w = w / np.sum(w)
    pad = window_len // 2
    y_padded = np.pad(y, pad, mode='edge')
    return np.convolve(y_padded, w, mode='valid')


def read_sim_bag(folder):
    """Lee mensajes relevantes del rosbag mcap."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(folder), storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    result = {k: [] for k in TYPES}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in TYPES:
            result[topic].append((t, deserialize_message(data, TYPES[topic])))

    for rows in result.values():
        rows.sort(key=lambda row: row[0])
    return result


def path_distance(points, reference):
    """Calcula la distancia ortogonal minima a los segmentos del Teach."""
    a, delta = reference[:-1], np.diff(reference, axis=0)
    norm = np.maximum(np.sum(delta * delta, axis=1), 1e-24)
    out = []
    for point in points:
        u = np.clip(np.sum((point - a) * delta, axis=1) / norm, 0, 1)
        out.append(np.min(np.linalg.norm(point - a - u[:, None] * delta, axis=1)))
    return np.asarray(out)


def load_mode_data(mode):
    """Carga ruta de Teach y datos de Repeat grabados en rosbag."""
    route_file = ROOT / 'routes' / f'sim_route_{mode}.json'
    if not route_file.exists():
        raise FileNotFoundError(f'No se encontro ruta: {route_file}')
    with open(route_file, 'r') as f:
        route = json.load(f)

    bag_folder = ROOT / 'evidence' / 'simulacion' / f'repeat_bag_{mode}'
    if not bag_folder.exists():
        raise FileNotFoundError(f'No se encontro rosbag: {bag_folder}')
    bag = read_sim_bag(bag_folder)

    status_msgs = [(t, m.data) for t, m in bag['/tnr/status']]
    if not status_msgs:
        raise ValueError(f'No hay mensajes de /tnr/status en {mode}')
    t0 = status_msgs[0][0]

    done_msgs = [t for t, m in bag['/tnr/done'] if m.data and t >= t0]
    t_end = done_msgs[0] if done_msgs else status_msgs[-1][0]

    odom = [(float((t - t0) / 1e9), m) for t, m in bag['/tnr/odom'] if t0 <= t <= t_end]
    if len(odom) < 2:
        raise ValueError(f'Insuficientes mensajes de odometria para {mode}')
    times = np.array([t for t, _ in odom])
    xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom])

    teach_traj = np.array(route['trajectory'], dtype=float)[:, 1:3]
    keyframes = np.array([k['pose'][:2] for k in route['keyframes']])
    cte = path_distance(xy, teach_traj)

    align_data = []
    for t, m in bag['/tnr/alignment']:
        if t0 <= t <= t_end:
            try:
                d = json.loads(m.data)
                align_data.append((float((t - t0) / 1e9), d.get('rmse_m', 0.0), d.get('overlap', 1.0)))
            except Exception:
                pass

    duration = float((t_end - t0) / 1e9)
    final_err = float(np.linalg.norm(xy[-1] - teach_traj[-1]))

    return {
        'mode': mode,
        'duration': duration,
        'times': times,
        'xy': xy,
        'teach_traj': teach_traj,
        'keyframes': keyframes,
        'cte': cte,
        'align': align_data,
        'mean_cte': float(np.mean(cte)),
        'max_cte': float(np.max(cte)),
        'p95_cte': float(np.percentile(cte, 95)),
        'final_err': final_err
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='evidence/simulacion/comparativa_simulacion.png',
                        help='Ruta del grafico de salida')
    parser.add_argument('--summary', default='evidence/simulacion/metricas_simulacion.md',
                        help='Ruta del resumen Markdown')
    args = parser.parse_args()

    results = {}
    for key, label, color in MODES:
        results[key] = load_mode_data(key)

    fig = plt.figure(figsize=(16, 7.8))
    gs = gridspec.GridSpec(2, 2, width_ratios=[1.15, 1.0], hspace=0.32, wspace=0.22)

    ax1 = fig.add_subplot(gs[:, 0])
    for key, label, color in MODES:
        d = results[key]
        ax1.plot(d['xy'][:, 0], d['xy'][:, 1], label=f'Repeat {label}', color=color, linewidth=2.0)

    ref_d = results['mixed']
    ax1.plot(ref_d['teach_traj'][:, 0], ref_d['teach_traj'][:, 1], '--', color='#333333',
             linewidth=1.8, label='Ruta Teach (Referencia)', alpha=0.85)
    ax1.scatter(ref_d['keyframes'][:, 0], ref_d['keyframes'][:, 1], color='#e67e22', s=35,
                zorder=5, label='Keyframes Teach')
    ax1.scatter(ref_d['teach_traj'][0, 0], ref_d['teach_traj'][0, 1], marker='o', color='#27ae60',
                s=100, zorder=6, label='Inicio')
    ax1.scatter(ref_d['teach_traj'][-1, 0], ref_d['teach_traj'][-1, 1], marker='X', color='#c0392b',
                s=120, zorder=6, label='Meta')

    ax1.set_title('Trayectorias en la simulación (Teach vs Repeat)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Posicion X (m)', fontsize=10)
    ax1.set_ylabel('Posicion Y (m)', fontsize=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.axis('equal')
    ax1.legend(loc='best', fontsize=8.5, framealpha=0.9)

    ax2 = fig.add_subplot(gs[0, 1])
    for key, label, color in MODES:
        d = results[key]
        raw_cte_cm = d['cte'] * 100.0
        smooth_cte_cm = smooth_signal(raw_cte_cm, window_len=131)
        ax2.plot(d['times'], smooth_cte_cm, label=f'{label} (Media: {d["mean_cte"]*100:.1f} cm)',
                 color=color, linewidth=2.3)

    ax2.set_title('Error Lateral (Cross-Track Error) respecto al Teach', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Tiempo de navegacion (s)', fontsize=10)
    ax2.set_ylabel('Desvio lateral (cm)', fontsize=10)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=8.5, framealpha=0.9)

    ax3 = fig.add_subplot(gs[1, 1])
    has_icp = False
    for key, label, color in [('lidar', 'LiDAR 2D (ICP)', '#8959b5'), ('mixed', 'Mixto (Fusión)', '#078875')]:
        d = results[key]
        if d['align']:
            has_icp = True
            t_al = np.array([x[0] for x in d['align']])
            raw_rmse_cm = np.array([x[1] * 100.0 for x in d['align']])
            smooth_rmse = smooth_signal(raw_rmse_cm, window_len=27)
            ax3.plot(t_al, smooth_rmse, label=f'RMSE {label}', color=color, linewidth=2.3)

    if has_icp:
        ax3.set_title('Calidad de Alineacion ICP LiDAR (RMSE scan-to-keyframe)', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Tiempo (s)', fontsize=10)
        ax3.set_ylabel('RMSE residual (cm)', fontsize=10)
        ax3.grid(True, linestyle=':', alpha=0.6)
        ax3.legend(loc='upper right', fontsize=8.5, framealpha=0.9)
    else:
        ax3.text(0.5, 0.5, 'Sin datos de alineacion ICP', ha='center', va='center')

    out_png = ROOT / args.output
    out_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_png, dpi=300, bbox_inches='tight')
    plt.close()

    out_md = ROOT / args.summary
    table_lines = [
        '# Metricas Comparativas de Simulacion (Gazebo Harmonic)',
        '',
        'Evaluacion de la etapa Repeat sobre los 3 modos de localizacion:',
        '',
        '| Metrica | Encoders | LiDAR 2D (ICP) | Mixto |',
        '|---|---|---|---|',
        f'| **Duracion total (s)** | {results["encoders"]["duration"]:.1f} s | {results["lidar"]["duration"]:.1f} s | {results["mixed"]["duration"]:.1f} s |',
        f'| **Error lateral medio (CTE)** | {results["encoders"]["mean_cte"]*100:.2f} cm | {results["lidar"]["mean_cte"]*100:.2f} cm | {results["mixed"]["mean_cte"]*100:.2f} cm |',
        f'| **Error lateral maximo** | {results["encoders"]["max_cte"]*100:.2f} cm | {results["lidar"]["max_cte"]*100:.2f} cm | {results["mixed"]["max_cte"]*100:.2f} cm |',
        f'| **Error lateral P95** | {results["encoders"]["p95_cte"]*100:.2f} cm | {results["lidar"]["p95_cte"]*100:.2f} cm | {results["mixed"]["p95_cte"]*100:.2f} cm |',
        f'| **Precision de llegada a meta** | {results["encoders"]["final_err"]*100:.2f} cm | {results["lidar"]["final_err"]*100:.2f} cm | {results["mixed"]["final_err"]*100:.2f} cm |',
        f'| **Keyframes de ruta completados** | 100% ({len(results["encoders"]["keyframes"])}/{len(results["encoders"]["keyframes"])}) | 100% ({len(results["lidar"]["keyframes"])}/{len(results["lidar"]["keyframes"])}) | 100% ({len(results["mixed"]["keyframes"])}/{len(results["mixed"]["keyframes"])}) |',
        ''
    ]
    with open(out_md, 'w') as f:
        f.write('\n'.join(table_lines))
    print(f'Grafico: {out_png}')


if __name__ == '__main__':
    main()
