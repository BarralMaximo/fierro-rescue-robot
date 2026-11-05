"""Precisión de la detección de víctimas (detector.py) en escenas sintéticas, sin ROS ni simulador.

Cada escena pone cajas de la competencia (0.25 x 0.25 x 0.30 m, colores de los .world: rojas = víctimas, azules =
señuelos) frente al robot, simula el scan del lidar (1080 rayos, ruido de 1 cm) y la foto de la cámara (320x240,
intrínsecos y montajes del contrato del robot real), y pasa las detecciones por DetectionClusterer como el nodo.
Mide dos cosas, porque las dos cuestan puntos:
    - error: distancia del grupo más cercano a cada caja roja. El juez da 100 puntos a <= 0.25 m y 50 a <= 0.50 m;
      los umbrales de acá son más exigentes porque se le suman el error del localizer y el de la TF.
    - falsos: detecciones en grupos lejos de toda caja roja. Un grupo así puede terminar en un reporte, que gasta
      uno de los N y tira el bono.

Los casos marcados xfail fallan hoy a propósito: son las mejoras pendientes, con el motivo.
Tabla por escenario (200 semillas): python3 test/test_detector.py
"""
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from fierro.core import DetectionClusterer
from fierro.detector import VisionProcessor, mask_red, laser_points_to_pixels
from fierro.perception import THRESHOLDS

PERCEPTION = yaml.safe_load((Path(__file__).parents[1] / 'params' / 'params.yaml').read_text())['perception']['ros__parameters']
VISION = {k: PERCEPTION[k] for k in THRESHOLDS} # umbrales del detector: los de params.yaml del repo

# Cámara del robot real (contrato del simulador: 320x240, sin distorsión)
W, H, FX, FY, CX, CY = 320, 240, 271.809, 271.809, 158.719, 120.092
# Montajes respecto de base_footprint (base_link está 0.0714 m más arriba)
LASER_T = np.array([0.0435, 0.0000526, 0.0714 + 0.11])
LASER_R = Rotation.from_euler('z', math.pi).as_matrix()
COLOR_T = np.array([0.054937, 0.025094, 0.0714 + 0.037410])
COLOR_R = Rotation.from_euler('xyz', [-0.000336, 0.004697, 0.003574]).as_matrix()
OPTICAL_R = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()  # color -> óptico: z adelante, x derecha, y abajo
# TF laser_link -> cam_1_color_optical_frame, la que el nodo le pasa al detector
R_OL = (COLOR_R @ OPTICAL_R).T @ LASER_R
T_OL = (COLOR_R @ OPTICAL_R).T @ (LASER_T - COLOR_T)

SIDE, HEIGHT = 0.25, 0.30
RED, BLUE = (0.8, 0.05, 0.05), (0.05, 0.05, 0.8)  # diffuse RGB de cuadrado_rojo y cuadrado_azul
ANGLES = np.linspace(-math.pi, math.pi, 1080)
GROUP_RADIUS = PERCEPTION['group_radius']  # más lejos que esto de una caja roja, el grupo es otra "caja"
SEEDS = range(10)


def camera(lag=0.0):
    """Pose del frame óptico en base_footprint. lag: cuánto giró el robot entre el scan y la foto (rad)."""
    yaw = Rotation.from_euler('z', lag).as_matrix()
    return yaw @ COLOR_R @ OPTICAL_R, yaw @ COLOR_T


def rays_against_box(o, d, c, yaw):
    """Distancia por cada rayo (origen o, direcciones d) hasta la caja de centro c; inf si no la toca."""
    cg, sg = math.cos(yaw), math.sin(yaw)
    to_box = np.array([[cg, sg], [-sg, cg]])
    oc, dc = to_box @ (o - c), d @ to_box.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (-SIDE / 2 - oc) / dc, (SIDE / 2 - oc) / dc
    near, far = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
    return np.where((near <= far) & (near > 0), near, np.inf)


def box(rng, dist=(0.5, 3.0), heading=0.35, yaw=None, color=RED):
    """Una caja al azar: dist (m) y heading (rad) de su centro visto desde el robot; yaw None = al azar."""
    r = rng.uniform(-heading, heading) if np.isscalar(heading) else rng.choice([-1, 1]) * rng.uniform(*heading)
    return dict(dist=rng.uniform(*dist), heading=r, yaw=rng.uniform(0, math.pi / 2) if yaw is None else r + yaw, color=color)


def other_box(rng, first, **kw):
    """Una segunda caja a más de 0.5 m de la primera: no se pisan."""
    while True:
        k = box(rng, **kw)
        ca, cb = (np.array([c['dist'] * math.cos(c['heading']), c['dist'] * math.sin(c['heading'])]) for c in (first, k))
        if np.hypot(*(ca - cb)) > 0.5:
            return k


def scene(rng, boxes, wall=None, lag=0.0, noise=0.01):
    """Foto BGR, rangos del lidar y centros reales (base_footprint) de las cajas rojas que el lidar ve.

    wall: distancia de una pared detrás de la primera caja, perpendicular a la línea de vista (o None).
    lag: cuánto giró el robot entre el scan y la foto (rad).
    """
    centers = [k['dist'] * np.array([math.cos(k['heading']), math.sin(k['heading'])]) for k in boxes]

    # Lidar: rayos desde laser_link, que está girado 180°
    o = LASER_T[:2]
    d = np.column_stack([np.cos(ANGLES + math.pi), np.sin(ANGLES + math.pi)])
    hits = [rays_against_box(o, d, c, k['yaw']) for c, k in zip(centers, boxes)]
    ranges = np.min(hits, axis=0)
    if wall is not None:
        n = centers[0] / np.linalg.norm(centers[0])
        with np.errstate(divide='ignore'):
            t = (centers[0] + n * wall - o) @ n / (d @ n)
        ranges = np.minimum(ranges, np.where(t > 0, t, np.inf))
    visible = [np.sum(np.isfinite(h) & (h <= ranges)) >= 3 for h in hits]  # una caja tapada no se le puede exigir
    ranges = ranges + rng.normal(0, noise, len(ranges))

    # Foto: fondo gris (paredes claras, piso oscuro) y la silueta de cada caja (convexa), de la más lejana a la más cercana
    img = np.empty((H, W, 3), np.float64)
    img[: int(CY)] = rng.uniform(160, 230)
    img[int(CY):] = rng.uniform(80, 140)
    r_bo, t_bo = camera(lag)
    for c, k in sorted(zip(centers, boxes), key=lambda ck: -ck[1]['dist']):
        cg, sg = math.cos(k['yaw']), math.sin(k['yaw'])
        base = np.array([[sx * SIDE / 2, sy * SIDE / 2] for sx in (-1, 1) for sy in (-1, 1)]) @ np.array([[cg, sg], [-sg, cg]]) + c
        p = (np.vstack([np.column_stack([base, np.full(4, z)]) for z in (0.0, HEIGHT)]) - t_bo) @ r_bo
        uv = np.column_stack([FX * p[:, 0] / p[:, 2] + CX, FY * p[:, 1] / p[:, 2] + CY])
        light = rng.uniform(0.55, 1.0)
        cv2.fillConvexPoly(img, cv2.convexHull(np.round(uv).astype(np.int32)), [255 * light * v for v in k['color'][::-1]])
    img = cv2.GaussianBlur(img + rng.normal(0, 4, img.shape), (3, 3), 0)
    red = [c for c, k, seen in zip(centers, boxes, visible) if k['color'] == RED and seen]
    return np.clip(img, 0, 255).astype(np.uint8), ranges, red


def groups(img, ranges):
    """Lo que hace el nodo con una escena: detecciones -> base_footprint -> DetectionClusterer. [(centro, n)]."""
    det = VisionProcessor(**VISION).extract_box_centers(img, ranges, ANGLES, R_OL, T_OL, FX, FY, CX, CY)
    cl = DetectionClusterer(GROUP_RADIUS)
    for x, y in np.asarray(det).reshape(-1, 2) @ LASER_R[:2, :2].T + LASER_T[:2]:  # laser_link -> base_footprint
        cl.add(x, y)
    return [(np.array(cl.center(g)), len(g['obs'])) for g in cl.groups]


def measure(gs, red):
    """(error, false_hits): peor distancia de una caja roja a su grupo más cercano, y detecciones en grupos lejos de toda caja roja."""
    error = max((min((np.hypot(*(c - r)) for c, _ in gs), default=math.inf) for r in red), default=0.0)
    false_hits = sum(n for c, n in gs if all(np.hypot(*(c - r)) > GROUP_RADIUS for r in red))
    return float(error), false_hits


SCENARIOS = {  # nombre: semilla -> argumentos de escena
    'frontal': lambda rng: dict(boxes=[box(rng, yaw=0.0)]),
    'rotated': lambda rng: dict(boxes=[box(rng)]),
    'wall behind': lambda rng: dict(boxes=[box(rng, dist=(0.5, 2.5))], wall=rng.uniform(0.05, 0.6)),
    'photo edge': lambda rng: dict(boxes=[box(rng, dist=(0.7, 2.5), heading=(0.45, 0.52))], wall=rng.uniform(0.2, 1.0)),
    'box and decoy': lambda rng: dict(boxes=[k := box(rng, dist=(0.6, 2.5)), other_box(rng, k, dist=(0.6, 2.5), color=BLUE)]),
    'two red boxes': lambda rng: dict(boxes=[k := box(rng, dist=(0.6, 2.5)), other_box(rng, k, dist=(0.6, 2.5))]),
    'scan-photo lag': lambda rng: dict(boxes=[box(rng, dist=(0.5, 2.5))], wall=rng.uniform(0.2, 1.0),
                                          lag=rng.choice([-1, 1]) * math.radians(rng.uniform(3, 6))),
}


def evaluate(name, seed):
    rng = np.random.default_rng(seed)
    img, ranges, red = scene(rng, **SCENARIOS[name](rng))
    return measure(groups(img, ranges), red)


# ---------- piezas sueltas ----------

def test_projection_optical_axis_and_behind():
    """Un punto sobre el eje óptico cae en (cx, cy); uno detrás de la cámara queda marcado."""
    r_bo, t_bo = camera()
    optical = np.array([[0, 0, 1.0], [0, 0, -1.0]])
    xyz = (optical @ r_bo.T + t_bo - LASER_T) @ LASER_R  # óptico -> base_footprint -> laser_link
    u, v, in_front = laser_points_to_pixels(xyz, R_OL, T_OL, FX, FY, CX, CY)
    assert u[0] == pytest.approx(CX) and v[0] == pytest.approx(CY)
    assert in_front.tolist() == [True, False]


def _is_red(bgr):
    p = VisionProcessor(**VISION)
    m = mask_red(cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV), p.red_hue_low_1, p.red_hue_high_1,
                     p.red_hue_low_2, p.red_hue_high_2, p.min_saturation, p.min_value)
    return m[0, 0] == 255


@pytest.mark.parametrize('light', [0.45, 0.7, 1.0])
def test_mask_takes_the_red_box_under_different_light(light):
    assert _is_red([255 * light * k for k in RED[::-1]])


@pytest.mark.parametrize('bgr', [[204, 13, 13], [180, 180, 180], [40, 40, 40], [30, 120, 230]],
                         ids=['blue decoy', 'gray wall', 'shadow', 'orange'])
def test_mask_ignores_other_colors(bgr):
    assert not _is_red(bgr)


@pytest.mark.parametrize('light', [0.3, 0.6, 1.0, 1.3])
def test_mask_ignores_the_simulator_wood(light):
    # Paredes de los .world: diffuse 0.55 0.35 0.2, hue ~13 y S ~162: solo el hue la separa del rojo
    assert not _is_red(np.clip([255 * light * k for k in (0.2, 0.35, 0.55)], 0, 255))


# ---------- escenas completas ----------

OCCLUDED = pytest.mark.xfail(
    reason='Con una caja tapada en parte por otra, el lidar ve solo un costado de la cara y sumar 12.5 cm a lo largo '
           'de cada rayo corre el centro de costado (hasta ~20 cm). Ajustar la recta de la cara y correrse por su '
           'normal (detector.py, problema 6)', strict=False)


@pytest.mark.parametrize('name, max_error', [
    ('frontal', 0.05), ('rotated', 0.06), ('wall behind', 0.06), ('photo edge', 0.15),
    ('box and decoy', 0.06), pytest.param('two red boxes', 0.06, marks=OCCLUDED),
])
@pytest.mark.parametrize('seed', SEEDS)
def test_precision(name, max_error, seed):
    assert evaluate(name, seed)[0] < max_error


@pytest.mark.parametrize('name', ['frontal', 'rotated', 'wall behind', 'photo edge', 'box and decoy',
                                    'two red boxes'])
@pytest.mark.parametrize('seed', SEEDS)
def test_no_false_groups(name, seed):
    """Sin la erosión y los tramos del detector, los rayos de la pared que rozan la caja daban falsos (58 % con pared detrás)."""
    assert evaluate(name, seed)[1] == 0


@pytest.mark.xfail(reason='La foto y el scan no se sincronizan: con 3-6° de giro entre ambos, rayos de la pared caen '
                          'sobre el rojo de la caja y forman grupos falsos (detector.py, problema 7)', strict=False)
@pytest.mark.parametrize('seed', SEEDS)
def test_lag_between_scan_and_photo(seed):
    error, false_hits = evaluate('scan-photo lag', seed)
    assert error < 0.06 and false_hits == 0


@pytest.mark.parametrize('seed', SEEDS)
def test_lone_blue_decoy_is_not_detected(seed):
    rng = np.random.default_rng(seed)
    img, ranges, _ = scene(rng, [box(rng, color=BLUE)])
    assert groups(img, ranges) == []


if __name__ == '__main__':
    print(f'{"escenario":20}{"mediana":>9}{"p90":>8}{"máx":>8}{"sin ver":>9}{"con falsos":>12}{"falsos/escena":>15}'
          '   (200 semillas; error en cm)')
    for name in SCENARIOS:
        r = np.array([evaluate(name, s) for s in range(200)])
        e, f = r[:, 0], r[:, 1]
        ok = e[np.isfinite(e)]
        print(f'{name:20}{100 * np.median(ok):9.1f}{100 * np.percentile(ok, 90):8.1f}{100 * ok.max():8.1f}'
              f'{np.mean(~np.isfinite(e)):9.0%}{np.mean(f > 0):12.0%}{f.mean():15.1f}')
