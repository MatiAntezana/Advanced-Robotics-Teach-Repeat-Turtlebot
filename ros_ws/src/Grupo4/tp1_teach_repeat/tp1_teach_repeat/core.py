"""Matemática SE(2), encoders, ICP Open3D y persistencia, sin dependencia de ROS.

T_a_b transforma coordenadas de b a a. Las poses son (x, y, yaw), en m y rad.
"""
import json
import math
import os
from pathlib import Path

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c*b[0] - s*b[1],
                     a[1] + s*b[0] + c*b[1], wrap(a[2] + b[2])])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c*a[0] - s*a[1], s*a[0] - c*a[1], -a[2]])


def between(a, b):
    return compose(inverse(a), b)


def matrix(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([[c, -s, 0., a[0]], [s, c, 0., a[1]],
                     [0., 0., 1., 0.], [0., 0., 0., 1.]])


def transform_points(points, pose):
    return (matrix(pose)[:3, :3] @ np.asarray(points).T).T + matrix(pose)[:3, 3]


def wheel_step(left, right, radius, separation):
    """Exponencial exacta del movimiento diferencial, incluso para giro puro."""
    ds = radius * (right + left) / 2
    da = radius * (right - left) / separation
    scale = float(np.sinc(da / (2*math.pi)))
    return np.array([ds*scale*math.cos(da/2), ds*scale*math.sin(da/2), da])


def interpolate(history, stamp):
    """Interpola encoders al tiempo del láser; nunca extrapola al futuro."""
    if not history or stamp < history[0][0] or stamp > history[-1][0]:
        return None
    for (ta, a), (tb, b) in zip(history, list(history)[1:]):
        if ta <= stamp <= tb:
            f = (stamp-ta) / (tb-ta) if tb > ta else 0.
            return np.array([a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1]),
                             wrap(a[2]+f*wrap(b[2]-a[2]))])
    return history[-1][1].copy()


def alignment_quality(source, target, pose=(0., 0., 0.), distance=.25):
    """Residual scan→referencia y solapamiento; no es error de pose/ground truth."""
    if len(source) < 20 or len(target) < 20:
        return dict(rmse_m=None, p95_m=None, overlap=0.)
    import open3d as o3d
    clouds = []
    for points in (transform_points(source, pose), np.asarray(target)):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
        clouds.append(cloud)
    distances = np.asarray(clouds[0].compute_point_cloud_distance(clouds[1]))
    valid = np.isfinite(distances) & (distances <= distance)
    return dict(rmse_m=float(np.sqrt(np.mean(distances[valid]**2))) if valid.any() else None,
                p95_m=float(np.percentile(distances, 95)), overlap=float(np.mean(valid)))


class ICP:
    """Point-to-point Open3D, nubes z=0 y proyección explícita a SE(2)."""
    def __init__(self, distance=0.25, fitness=0.55, rmse=0.10):
        import open3d as o3d
        self.o3d = o3d
        self.distance, self.fitness, self.rmse = distance, fitness, rmse

    def align(self, source, target, guess):
        if len(source) < 20 or len(target) < 20:
            return None
        o = self.o3d
        clouds = []
        for points in (source, target):
            p = o.geometry.PointCloud()
            p.points = o.utility.Vector3dVector(np.asarray(points, dtype=float))
            clouds.append(p)
        r = o.pipelines.registration.registration_icp(
            clouds[0], clouds[1], self.distance, matrix(guess),
            o.pipelines.registration.TransformationEstimationPointToPoint(),
            o.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        t = r.transformation
        if (r.fitness < self.fitness or r.inlier_rmse > self.rmse
                or not np.isfinite(t).all() or t[2, 2] < 0.99):
            return None
        return np.array([t[0, 3], t[1, 3], math.atan2(t[1, 0], t[0, 0])])


def polar_command(error, position_tolerance=0.035, yaw_tolerance=0.06,
                  max_v=0.18, max_w=1.0, k_rho=0.9, k_alpha=2.5,
                  k_beta=-0.6, k_yaw=1.5):
    x, y, yaw = error
    rho = math.hypot(x, y)
    if rho <= position_tolerance:
        return (0., 0.) if abs(yaw) <= yaw_tolerance else (
            0., float(np.clip(k_yaw*yaw, -max_w, max_w)))
    alpha = math.atan2(y, x)
    # Permite repetir segmentos enseñados marcha atrás, sin vueltas de 180°.
    direction = 1.
    if abs(alpha) > math.pi/2:
        direction = -1.
        alpha = wrap(alpha - math.copysign(math.pi, alpha))
    beta = wrap(yaw - alpha)
    v = direction*min(max_v, k_rho*rho)*max(0., math.cos(alpha))
    if abs(alpha) > 1.25:
        v = 0.
    w = float(np.clip(k_alpha*alpha + k_beta*beta, -max_w, max_w))
    return v, w


def save_route(path, frames, mode, trajectory):
    """Reemplazo atómico: un corte no deja un JSON parcialmente escrito."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    data = dict(version=1, frame='odom', mode=mode, keyframes=frames,
                trajectory=trajectory)
    with tmp.open('w') as f:
        json.dump(data, f, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)


def load_route(path, require_scans=False):
    with Path(path).expanduser().open() as f:
        data = json.load(f)
    if data.get('version') != 1 or data.get('frame') != 'odom':
        raise ValueError('Formato de ruta no soportado')
    frames = data.get('keyframes', [])
    if not frames:
        raise ValueError('La ruta no contiene keyframes')
    for i, frame in enumerate(frames):
        p = np.asarray(frame['pose'], dtype=float)
        points = np.asarray(frame.get('points', []), dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError(f'Pose inválida en keyframe {i}')
        if points.size and (points.ndim != 2 or points.shape[1] != 3
                            or not np.isfinite(points).all()):
            raise ValueError(f'Nube inválida en keyframe {i}')
        if require_scans and len(points) < 20:
            raise ValueError(f'Keyframe {i} sin escaneo: enseñar con LiDAR disponible')
    return data
