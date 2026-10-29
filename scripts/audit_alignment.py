#!/usr/bin/env python3
"""Calcula el residuo RMSE entre escaneos de keyframes vecinos en una ruta."""
import argparse
import json
from pathlib import Path

import numpy as np
from tp1_teach_repeat.core import alignment_quality, load_route, transform_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('route')
    parser.add_argument('--output')
    args = parser.parse_args()
    frames = load_route(args.route, require_scans=True)['keyframes']
    clouds = [transform_points(f['points'], f['pose']) for f in frames]
    results = []
    for i, (frame, cloud) in enumerate(zip(frames, clouds)):
        neighbors = [j for j, f in enumerate(frames) if j != i
                     and np.linalg.norm(np.asarray(frame['pose'])[:2]-np.asarray(f['pose'])[:2]) < .8]
        if neighbors:
            results.append(dict(keyframe=i, neighbors=neighbors,
                                **alignment_quality(cloud, np.concatenate([clouds[j] for j in neighbors]))))
    valid = [r['rmse_m'] for r in results if r['rmse_m'] is not None]
    report = dict(route=str(Path(args.route).absolute()), metric='scan_to_neighbor_rmse',
                  mean_rmse_m=float(np.mean(valid)) if valid else None,
                  max_rmse_m=float(np.max(valid)) if valid else None, keyframes=results)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text+'\n')


if __name__ == '__main__':
    main()
