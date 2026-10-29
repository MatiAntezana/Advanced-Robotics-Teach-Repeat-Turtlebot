#!/usr/bin/env python3
"""Grafica las trayectorias de validacion guardadas por validate_gazebo.py."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', default='validation/gazebo')
    args = parser.parse_args()
    folder = Path(args.directory)
    results = json.loads((folder/'results.json').read_text())
    fig, axes = plt.subplots(1, len(results), figsize=(5*len(results), 4.5), squeeze=False)
    for ax, result in zip(axes[0], results):
        mode = result['mode']
        data = json.loads((folder/f'{mode}_trajectories.json').read_text())
        route = json.loads((folder/f'{mode}_route.json').read_text())
        for stage, color in [('teach', '#bd751a'), ('repeat', '#008e87')]:
            points = np.array(data[stage])
            ax.plot(points[:, 1], points[:, 2], label=stage.title(), color=color, linewidth=2)
        keyframes = np.array([k['pose'] for k in route['keyframes']])
        ax.scatter(keyframes[:, 0], keyframes[:, 1], color='#313e4b', s=25, label='Keyframes', zorder=3)
        for i, (x, y, _) in enumerate(keyframes):
            ax.annotate(str(i), (x, y), xytext=(4, 5 + 11*(i % 2)), textcoords='offset points', fontsize=8)
        ax.set_title(f"{mode}\nError final: {result['final_position_error_m']*100:.1f} cm")
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8)
    fig.suptitle('Validacion Teach y Repeat en Gazebo', fontsize=12, fontweight='bold')
    fig.tight_layout()
    target = folder/'trajectories.png'
    fig.savefig(target, dpi=180)
    print(f"Grafico guardado en {target.absolute()}")


if __name__ == '__main__':
    main()
