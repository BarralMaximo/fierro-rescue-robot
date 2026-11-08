"""
Demo: una misión completa de Fierro en un laberinto simulado, para la página de GitHub Pages y la animación del README.

Corre Mission, el mismo código que maneja el robot, en el simulador cinemático de test_mission.py: el lidar contra el
mundo (paredes y cajas que el mapa no tiene), la percepción con la regla de DetectionClusterer y motores con el retardo,
la zona muerta y el giro desparejo del robot real. Guarda la repetición en docs/run.js, que dibuja docs/index.html. Es
un script y no un JSON para que la página ande abierta directo, sin servidor (los navegadores no dejan leer archivos
con fetch desde file://).

    python3 docs/demo.py                      # docs/run.js
    python3 docs/demo.py --gif docs/demo.gif  # además, la animación del README

Sin ROS: alcanza con numpy, scipy, pyyaml y pytest (los del simulador de los tests); matplotlib, solo para el GIF.
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'test')]

from test_mission import ANG, P, in_camera, lidar, real_motors, with_box  # noqa: E402

from fierro.core import Board, normalize  # noqa: E402
from fierro.mission import Mission  # noqa: E402

# 6 x 6 m con pasillos de 0.9 m, como los de práctica. Cada celda son 4 caracteres de ancho y 2 líneas de alto.
MAZE = """
+---+---+---+---+---+---+
|       |               |
+   +   +   +---+---+   +
|   |   |   |       |   |
+   +   +   +   +   +   +
|   |       |   |       |
+   +---+---+   +---+---+
|       |               |
+---+   +   +---+---+   +
|       |   |       |   |
+   +---+   +   +   +   +
|               |       |
+---+---+---+---+---+---+
"""
CELL, RES, WALL = 1.0, 0.05, 0.10  # m
START = (0.0, 0.0, 0.0)  # centro de la celda de abajo a la izquierda, mirando a +x
VICTIMS = [(1.0, 2.72), (5.28, 3.5), (3.5, 1.3)]  # cajas rojas, apoyadas en paredes
DECOYS = [(2.0, -0.3), (3.5, 5.2)]  # cajas azules: el lidar las ve igual que a las víctimas
DT = 0.2  # s por ciclo de control
RAY_STEP = 8  # para la repetición se guarda un rayo de cada 8 (cada 2.7°)

def board(text):
    """El Board de un laberinto dibujado con +, - y |: paredes de WALL de espesor sobre una grilla de CELL."""
    rows = text.strip('\n').split('\n')
    n_i, n_j, size, t = (len(rows) - 1) // 2, (len(rows[0]) - 1) // 4, round(CELL / RES), round(WALL / RES / 2)
    g = np.zeros((n_i * size + 1, n_j * size + 1), np.int16)
    for r, line in enumerate(rows):
        for c, ch in enumerate(line):
            j = (c // 4) * size
            if r % 2 == 0 and ch == '-' and c % 4 == 2:  # pared horizontal
                i = (n_i - r // 2) * size
                g[max(0, i - t) : i + t, j : j + size + 1] = 100
            if r % 2 == 1 and ch == '|':  # pared vertical
                i = (n_i - 1 - r // 2) * size
                g[i : i + size + 1, max(0, j - t) : j + t] = 100
    return Board(g.ravel(), g.shape[1], g.shape[0], RES, -CELL / 2, -CELL / 2)

def run():
    """Corre la misión hasta /done. Devuelve (mapa, mundo, misión, cuadros, reportes); un cuadro por ciclo."""
    grid = board(MAZE)
    world = grid
    for box in VICTIMS + DECOYS: world = with_box(world, *box)

    m = Mission(grid, START[:2], P)
    motors = real_motors(dt=DT)
    x, y, yaw = START
    t, done, reports, views, frames = 0.0, False, [], {box: [] for box in VICTIMS}, []
    seen, last = np.zeros_like(m.seen), None
    while t < P.time_limit and not done:
        # Percepción simulada como en test_mission: /jar/red desde el segundo scan y el reporte a los ~0.8 s en cuadro
        red = [box for box, v in views.items() if len(v) * DT > 0.28 and box not in reports]
        scan = lidar(world, x, y)
        (vx, vy, w), done = m.step(x, y, yaw, t, scan, reports, len(VICTIMS), red)

        k = np.round((np.arctan2(scan[:, 1] - y, scan[:, 0] - x) - ANG[0]) / (ANG[1] - ANG[0])).astype(int) % len(ANG)
        ranges = np.zeros(len(ANG))
        ranges[k] = np.hypot(scan[:, 0] - x, scan[:, 1] - y)
        cands = [[*map(_cm, cd['xy']), cd['why'] or ('target' if cd is m.target else 'pending')] for cd in m.candidates]
        frame = {
            't': round(t, 1),
            'p': [_cm(x), _cm(y), round(yaw, 3)],
            's': m.state,
            'l': [_cm(r) for r in ranges[::RAY_STEP]],
            'v': np.flatnonzero(m.seen & ~seen).tolist(),
        }
        if cands != last: frame['c'] = last = cands
        if m.log:
            frame['m'] = m.log[:]
            m.log.clear()
        frames.append(frame)
        seen = m.seen.copy()

        vx, vy, w = motors(vx, vy, w)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, yaw = x + (c * vx - s * vy) * DT, y + (s * vx + c * vy) * DT, normalize(yaw + w * DT)
        t += DT
        for box, v in views.items():
            if box in reports or not in_camera(grid, x, y, yaw, *box, m.p): continue
            v.append((x, y))
            if len(v) * DT > 0.8:
                reports.append(box)
                frames[-1]['r'] = [[_cm(a), _cm(b)] for a, b in reports]
    return grid, world, m, frames, reports

def _cm(v):
    return int(round(float(v) * 100))

def save(grid, world, m, frames, reports):
    """Escribe run.js (el mapa, las cajas, el plan y los cuadros, en cm) al lado de index.html. Devuelve los datos."""
    h, w = grid.occupied.shape
    xy = np.array([f['p'][:2] for f in frames]) / 100
    data = {
        'repo': os.environ.get('GITHUB_REPOSITORY', 'BarralMaximo/fierro-rescue-robot'),
        'commit': os.environ.get('GITHUB_SHA'),
        'map': {'res': grid.res, 'ox': grid.ox, 'oy': grid.oy, 'w': w, 'h': h},
        'victims': [[_cm(a), _cm(b)] for a, b in VICTIMS],
        'decoys': [[_cm(a), _cm(b)] for a, b in DECOYS],
        'box': 25,
        'tour': [[_cm(a), _cm(b)] for a, b in m.points],
        'body': {'front': _cm(P.body_front), 'rear': _cm(P.body_rear), 'side': _cm(P.body_side)},
        'fov': P.camera_fov,
        'ray0': float(ANG[0]),
        'ray_step': float(ANG[RAY_STEP] - ANG[0]),
        'dt': DT,
        'summary': {
            'time': frames[-1]['t'],
            'victims': len(VICTIMS),
            'reported': len(reports),
            'done': frames[-1]['s'] == 'done',
            'clearance': _cm(world.distance(xy[:, 0], xy[:, 1]).min()),  # del centro del robot a lo más cercano
            'home': _cm(math.dist(xy[-1], START[:2])),
        },
        'frames': frames,
    }
    data['map']['walls'] = np.flatnonzero(grid.occupied).tolist()
    js = json.dumps(data, separators=(',', ':'))
    Path(__file__).with_name('run.js').write_text(f'const RUN = {js};\n')
    return data

def gif(path, data, step=2, size=480):
    """La animación del README con los mismos datos que la página: un cuadro cada `step` ciclos."""
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    from PIL import Image

    mp, frames, n = data['map'], data['frames'], len(data['victims'])
    ext = [mp['ox'], mp['ox'] + mp['w'] * mp['res'], mp['oy'], mp['oy'] + mp['h'] * mp['res']]
    cells = np.zeros(mp['w'] * mp['h'], bool)
    cells[mp['walls']] = True
    walls = np.zeros((mp['h'], mp['w'], 4))
    walls[cells.reshape(mp['h'], mp['w'])] = matplotlib.colors.to_rgba('#1f2937')
    seen, shade = np.zeros((mp['h'] * mp['w'], 4)), matplotlib.colors.to_rgba('#d1fae5')
    ring = {'reported': '#16a34a', 'pending': '#d97706', 'target': '#d97706'}

    fig = plt.figure(figsize=(size / 100, size / 100), dpi=100, facecolor='white')
    ax = fig.add_axes((0.02, 0.02, 0.96, 0.88))
    ax.set_axis_off()
    seen_img = ax.imshow(seen.reshape(mp['h'], mp['w'], 4), origin='lower', extent=ext, interpolation='nearest')
    ax.imshow(walls, origin='lower', extent=ext, interpolation='nearest')
    tour = np.array(data['tour']) / 100
    ax.plot(tour[:, 0], tour[:, 1], 'o', color='#9ca3af', ms=3)
    for boxes, color in ((data['victims'], '#dc2626'), (data['decoys'], '#2563eb')):
        for bx, by in np.array(boxes) / 100: ax.add_patch(Rectangle((bx - 0.125, by - 0.125), 0.25, 0.25, color=color))
    (trail,) = ax.plot([], [], '-', color='#0284c7', lw=1.6)
    (lidar_pts,) = ax.plot([], [], 's', color='#f97316', ms=1.6)
    cam = ax.add_patch(Polygon(np.zeros((3, 2)), color='#facc15', alpha=0.35, lw=0))
    body = ax.add_patch(Polygon(np.zeros((4, 2)), color='#111827'))
    title = fig.text(0.5, 0.95, '', ha='center', va='center', fontsize=12, color='#111827')
    rings, images = [], []
    xy = np.array([f['p'][:2] for f in frames]) / 100
    angles = data['ray0'] + data['ray_step'] * np.arange(len(frames[0]['l']))
    cands, reports, front, rear, side = [], [], *(data['body'][k] / 100 for k in ('front', 'rear', 'side'))
    for k, f in enumerate(frames):
        seen[f['v']] = shade
        cands, reports = f.get('c', cands), f.get('r', reports)
        if k % step and k != len(frames) - 1: continue
        x, y, yaw = xy[k, 0], xy[k, 1], f['p'][2]
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        seen_img.set_data(seen.reshape(mp['h'], mp['w'], 4))
        trail.set_data(xy[: k + 1, 0], xy[: k + 1, 1])
        r = np.array(f['l']) / 100
        lidar_pts.set_data((x + r * np.cos(angles))[r > 0], (y + r * np.sin(angles))[r > 0])
        arc = np.linspace(-data['fov'] / 2, data['fov'] / 2, 12)
        cam.set_xy(np.vstack([(front, 0), np.column_stack([front + 1.2 * np.cos(arc), 1.2 * np.sin(arc)])]) @ rot.T + (x, y))
        body.set_xy(np.array([(front, side), (-rear, side), (-rear, -side), (front, -side)]) @ rot.T + (x, y))
        for a in rings: a.remove()
        rings = [ax.add_patch(Circle((cx / 100, cy / 100), 0.24, fill=False, lw=2, color=ring.get(why, '#9ca3af'))) for cx, cy, why in cands]
        rings += [ax.add_patch(Circle((rx / 100, ry / 100), 0.3, fill=False, lw=2, color='#16a34a')) for rx, ry in reports]
        state = {'going': 'buscando', 'returning': 'volviendo a la largada', 'done': '/done en la largada'}[f['s']]
        title.set_text(f'{f["t"]:3.0f} s  ·  {state}  ·  víctimas {len(reports)}/{n}')
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()))
    plt.close(fig)

    # Una sola paleta para todos los cuadros: lo que no cambia queda igual y el GIF guarda solo lo que cambió
    sample = Image.new('RGB', (size, 3 * size))
    for i, im in enumerate((images[0], images[len(images) // 2], images[-1])): sample.paste(im, (0, i * size))
    palette = sample.quantize(colors=48, method=Image.Quantize.MEDIANCUT)
    images = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
    durations = [80] * (len(images) - 1) + [2500]
    images[0].save(path, save_all=True, append_images=images[1:], duration=durations, loop=0, optimize=True)

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--gif', type=Path, help='además, guarda la animación del README acá')
    args = ap.parse_args()
    data = save(*run())
    print(data['summary'])
    if args.gif: gif(args.gif, data)
