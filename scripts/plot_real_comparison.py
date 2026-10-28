#!/usr/bin/env python3
"""Compara registros reales sin mezclar fuentes de odometría ni asumir éxito.

Desde la raíz: pixi run --frozen python scripts/plot_real_comparison.py
Lee SQLite en modo solo lectura. Requiere el entorno ROS del proyecto.
"""
import argparse
import csv
import hashlib
import json
import re
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, String

ROOT = Path(__file__).resolve().parents[1]
MODES = [('encoders', 'Encoders', '#2671be'), ('lidar', 'LiDAR', '#8959b5'),
         ('mixed', 'Mixto', '#078875')]
TYPES = {'/tnr/odom': Odometry, '/tnr/setpoint': PoseStamped,
         '/tnr/status': String, '/tnr/done': Bool, '/tnr/alignment': String}


def read_bag(folder):
    result = {k: [] for k in TYPES}
    paths = sorted(folder.glob('*.db3'))
    if not paths:
        raise FileNotFoundError(f'No hay SQLite en {folder}')
    for path in paths:
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True) as db:
            for topic, stamp, data in db.execute(
                    'SELECT t.name,m.timestamp,m.data FROM messages m '
                    'JOIN topics t ON t.id=m.topic_id ORDER BY m.timestamp'):
                if topic in TYPES:
                    result[topic].append((stamp, deserialize_message(data, TYPES[topic])))
    for rows in result.values():
        rows.sort(key=lambda row: row[0])
    return result, paths


def path_distance(points, reference):
    """Distancia mínima a segmentos Teach, sin correspondencia temporal."""
    a, delta = reference[:-1], np.diff(reference, axis=0)
    norm = np.maximum(np.sum(delta * delta, axis=1), 1e-24)
    out = []
    for point in points:
        u = np.clip(np.sum((point - a) * delta, axis=1) / norm, 0, 1)
        out.append(np.min(np.linalg.norm(point - a - u[:, None] * delta, axis=1)))
    return np.asarray(out)


def load_mode(mode, max_duration=None):
    route_path = ROOT / 'routes' / f'lab_route_{mode}.json'
    route = json.loads(route_path.read_text())
    if route['mode'] != mode or route['frame'] != 'odom':
        raise ValueError(f'Ruta incompatible: {route_path}')
    bag, paths = read_bag(ROOT / 'evidence' / 'laboratorio' / f'repeat_bag_{mode}')
    status = [(t, m.data) for t, m in bag['/tnr/status']]
    if not status or any(not s.startswith(f'repeat:{mode}:') for _, s in status):
        raise ValueError(f'Status ausente o inconsistente: {mode}')
    t0 = status[0][0]
    done = [t for t, m in bag['/tnr/done'] if m.data and t >= t0]
    end = done[0] if done else status[-1][0]
    # Bool se publica justo antes del String de la misma actualización.
    # Incluye ese status final para conservar el índice de la meta.
    if done:
        final_status = next((t for t, s in status if t >= done[0] and s.endswith('done=True')), None)
        if final_status is None or final_status-done[0] > 500_000_000:
            raise ValueError(f'Bool y status final no coinciden: {mode}')
        end = final_status
    if max_duration is not None:
        end = min(end, t0 + round(max_duration * 1e9))

    def window(topic):
        return [(float((t-t0)/1e9), m) for t, m in bag[topic] if t0 <= t <= end]

    odom = window('/tnr/odom')
    if len(odom) < 2:
        raise ValueError(f'Odometría insuficiente: {mode}')
    xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for _, m in odom])
    times = np.array([t for t, _ in odom])
    # trajectory contiene [stamp_ns,x,y,yaw]; usa el Teach guardado, no el bag
    # de odometría del fabricante, cuyo origen y estimador son distintos.
    teach = np.asarray(route['trajectory'], dtype=float)[:, 1:3]
    kfs = np.array([k['pose'] for k in route['keyframes']])
    cross = path_distance(xy, teach)
    state = [(float((t-t0)/1e9), int(re.search(r'keyframe=(\d+)', s)[1]))
             for t, s in status if t <= end]
    if any(i >= len(kfs) for _, i in state):
        raise ValueError('Índice de referencia fuera de la ruta')
    # Verifica asociación bag/ruta mediante el objetivo global reconstruido.
    # Odom y setpoint de una publicación comparten sello de medición.
    def stamp(m):
        return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    by_stamp = {stamp(m): m for _, m in bag['/tnr/odom']}
    mismatch = []
    for _, goal in window('/tnr/setpoint'):
        o = by_stamp.get(stamp(goal))
        if o is None:
            continue
        p, q = o.pose.pose.position, o.pose.pose.orientation
        yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v = goal.pose.position
        # done publica objetivo nulo, que no define un keyframe guardado.
        if np.hypot(v.x, v.y) < 1e-10:
            continue
        target = [p.x+np.cos(yaw)*v.x-np.sin(yaw)*v.y,
                  p.y+np.sin(yaw)*v.x+np.cos(yaw)*v.y]
        mismatch.append(float(np.min(np.linalg.norm(kfs[:, :2]-target, axis=1))))
    if not mismatch or np.percentile(mismatch, 95) > .03:
        raise ValueError(f'No se verificó asociación de ruta: {mode}')
    align = [(t, json.loads(m.data)) for t, m in window('/tnr/alignment')]
    last_index = state[-1][1]
    summary = dict(mode=mode, completed_in_recording=bool(done),
                   completed_in_window=bool(done and done[0] <= end),
                   observed_window_s=(end-t0)/1e9,
                   completion_time_s=(done[0]-t0)/1e9 if done else None,
                   last_reference_index=last_index, final_reference_index=len(kfs)-1,
                   reference_index_percent=100*last_index/(len(kfs)-1),
                   teach_polyline_length_m=float(np.linalg.norm(np.diff(teach, axis=0), axis=1).sum()),
                   observed_cross_track_rms_m=float(np.sqrt(np.trapezoid(cross**2, times)/(times[-1]-times[0]))),
                   observed_cross_track_max_m=float(cross.max()),
                   route_match_p95_m=float(np.percentile(mismatch, 95)),
                   first_status_receipt_ns=t0, analysis_end_receipt_ns=end,
                   independent_ground_truth=False)
    clouds = []
    for k in route['keyframes']:
        pts = np.asarray(k.get('points', []))
        if pts.size:
            x, y, a = k['pose']
            rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
            clouds.append(pts[:, :2] @ rot.T + [x, y])
    return dict(summary=summary, xy=xy, times=times, teach=teach, kfs=kfs,
                teach_ns=np.array([p[0] for p in route['trajectory']], dtype=np.int64),
                keyframe_ns=np.array([k['stamp_ns'] for k in route['keyframes']], dtype=np.int64),
                state=np.array(state), cross=cross, align=align,
                cloud=np.concatenate(clouds) if clouds else np.empty((0, 2)),
                sources=[route_path, *paths])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'validation'/'real_comparison')
    parser.add_argument('--common-window', action='store_true',
                        help='Recortar los tres Repeat a la menor duración observada y encuadrar ese tramo')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = [load_mode(mode) for mode, _, _ in MODES]
    common = None
    if args.common_window:
        common = min(r['summary']['observed_window_s'] for r in results)
        results = [load_mode(mode, common) for mode, _, _ in MODES]
    for r in results:
        target = min(r['summary']['last_reference_index']+1, len(r['kfs'])-1)
        r['shown_kfs'] = r['kfs'][:target+1] if common is not None else r['kfs']
        mask = r['teach_ns'] <= r['keyframe_ns'][target]
        r['shown_teach'] = r['teach'][mask] if common is not None else r['teach']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titleweight': 'bold', 'axes.labelcolor': '#334155',
                         'text.color': '#142b40', 'axes.edgecolor': '#acb8c2',
                         'grid.color': '#dce3e9', 'grid.alpha': .75})
    fig = plt.figure(figsize=(18, 9 if common is not None else 11), facecolor='white')
    grid = fig.add_gridspec(2, 3, left=.055, right=.975,
                           bottom=.10,
                           top=.84,
                           height_ratios=[1 if common is not None else 1.55, 1],
                           wspace=.26, hspace=.55 if common is not None else .4)
    all_xy = np.vstack([r[k][:, :2] for r in results for k in ['shown_teach', 'xy', 'shown_kfs']])
    lo, hi = all_xy.min(axis=0)-.45, all_xy.max(axis=0)+.45
    # Una misma escala métrica para los tres mapas. No se alinea cada Repeat
    # a posteriori ni se recorta la trayectoria observada.
    for col, ((mode, name, color), r) in enumerate(zip(MODES, results)):
        ax = fig.add_subplot(grid[0, col])
        cloud = r['cloud']
        if len(cloud):
            cloud = cloud[::max(1, len(cloud)//12000)]
            ax.scatter(*cloud.T, s=1, c='#bfc7ce', alpha=.35, rasterized=True)
        ax.plot(*r['shown_teach'].T, '--', color='#596774', lw=1.7, label='Teach del tramo')
        ax.plot(*r['xy'].T, color=color, lw=2.4, label='Repeat estimado')
        ax.scatter(*r['shown_kfs'][:, :2].T, s=6, color='#596774', alpha=.5)
        ax.scatter(*r['xy'][0], marker='o', s=45, facecolor='white', edgecolor=color, zorder=5)
        ax.scatter(*r['xy'][-1], marker='X', s=80, color=color, zorder=5, label='Fin del segmento analizado')
        if common is None:
            ax.scatter(*r['kfs'][-1, :2], marker='*', s=130, color='#142b40', zorder=5, label='Meta Teach')
        s = r['summary']
        state = 'COMPLETADO' if s['completed_in_recording'] else 'REGISTRO PARCIAL · sin done=True'
        if common is not None:
            state = f'PRIMEROS {common:.1f} s DE REPEAT'
        ax.set_title(name if common is not None else f'{name}\n{state}',
                     fontsize=13, color=color, pad=17)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='x local [m]', ylabel='y local [m]')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, lw=.6)
        if col == 0:
            if common is not None:
                handles, labels = ax.get_legend_handles_labels()
                fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(.5, .925),
                           ncol=3, fontsize=10, frameon=False)
            else:
                ax.legend(loc='upper left', fontsize=8, framealpha=.94)
    axes = [fig.add_subplot(grid[1, col]) for col in range(3)]
    for (_, name, color), r in zip(MODES, results):
        axes[0].plot(r['times'], r['cross']*100, color=color, lw=1.7, label=name)
        n = r['summary']['final_reference_index']
        axes[1].step(r['state'][:, 0], 100*r['state'][:, 1]/n, where='post', color=color, lw=2, label=name)
        a = r['align']
        axes[2].plot([t for t, _ in a], [v['rmse_m']*100 for _, v in a], color=color, lw=1.5, label=name)
    axes[0].set(title='Distancia geométrica a Teach', ylabel='Distancia a la polilínea [cm]')
    axes[1].set(title='Avance por referencias', ylabel='Índice / índice final [%]', ylim=(-3, 104))
    if common is not None:
        axes[1].set_ylim(-1, max(r['summary']['reference_index_percent'] for r in results)+3)
    axes[2].set(title='Alineación escaneo–referencia', ylabel='RMSE publicado [cm]')
    for ax in axes:
        ax.set_xlabel('Tiempo desde primer status Repeat [s]')
        ax.grid(True, lw=.6)
        ax.legend(fontsize=9, loc='upper right')
        if common is not None:
            ax.set_xlim(0, common)
    fig.suptitle('Comparación de los 3 métodos (laboratorio)',
                 x=.5, y=.975, ha='center', fontsize=23, fontweight='bold')
    # Las trayectorias superiores quedan limpias; las métricas inferiores
    # conservan ejes, unidades y valores para poder interpretarlas.
    for ax in fig.axes[:3]:
        ax.set_xlabel('')
        ax.set_ylabel('')
        ax.tick_params(axis='both', which='both', bottom=False, top=False,
                       left=False, right=False, labelbottom=False, labelleft=False)
        for spine in ax.spines.values():
            spine.set_visible(False)
    fig.savefig(args.output/'comparacion_robot_real.png', dpi=190)
    plt.close(fig)
    summaries = [r['summary'] for r in results]
    with (args.output/'metricas.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    provenance = []
    for r in results:
        for p in r['sources']:
            with p.open('rb') as source:
                digest = hashlib.file_digest(source, 'sha256').hexdigest()
            provenance.append({'path': str(p.relative_to(ROOT)),
                               'sha256': digest})
    (args.output/'resultados.json').write_text(json.dumps(
        {'metrics': summaries, 'sources': provenance}, indent=2)+'\n')
    for s in summaries:
        print(json.dumps(s, ensure_ascii=False))
    print(f'Gráficos y métricas: {args.output.resolve()}')


if __name__ == '__main__':
    main()
