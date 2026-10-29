#!/usr/bin/env python3
"""Rota las poses y la trayectoria de una ruta guardada."""
import json
import math
import sys
from pathlib import Path
import numpy as np


def wrap(a):
    """Normaliza un angulo en el intervalo [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def align_route(input_path, output_path, angle_deg=7.20):
    """Aplica una rotacion 2D a los keyframes y trayectoria de la ruta."""
    with open(input_path, 'r') as f:
        data = json.load(f)

    alpha = math.radians(angle_deg)
    c, s = math.cos(alpha), math.sin(alpha)
    R = np.array([[c, -s], [s, c]])

    for kf in data.get('keyframes', []):
        p = np.array(kf['pose'])
        new_xy = R @ p[:2]
        new_yaw = wrap(p[2] + alpha)
        kf['pose'] = [float(new_xy[0]), float(new_xy[1]), float(new_yaw)]

    if 'trajectory' in data and data['trajectory']:
        new_traj = []
        for pt in data['trajectory']:
            t, x, y, yaw_val = pt[0], pt[1], pt[2], pt[3]
            new_xy = R @ np.array([x, y])
            new_yaw = wrap(yaw_val + alpha)
            new_traj.append([t, float(new_xy[0]), float(new_xy[1]), float(new_yaw)])
        data['trajectory'] = new_traj

    with open(output_path, 'w') as f:
        json.dump(data, f)
    print(f"Ruta rotada guardada en {output_path}")


if __name__ == '__main__':
    inp = sys.argv[1] if len(sys.argv) > 1 else 'routes/lab_route.json'
    out = sys.argv[2] if len(sys.argv) > 2 else inp
    angle = float(sys.argv[3]) if len(sys.argv) > 3 else 7.20
    align_route(inp, out, angle)
