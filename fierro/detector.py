import cv2
import numpy as np

from fierro.core import segments

def mask_red(hsv_image, hue_low_1, hue_high_1, hue_low_2, hue_high_2, min_saturation, min_value):
    """Máscara binaria (255 = rojo)."""
    low1 = np.array([hue_low_1, min_saturation, min_value])
    high1 = np.array([hue_high_1, 255, 255])
    low2 = np.array([hue_low_2, min_saturation, min_value])
    high2 = np.array([hue_high_2, 255, 255])
    mask1 = cv2.inRange(hsv_image, low1, high1)
    mask2 = cv2.inRange(hsv_image, low2, high2)
    return cv2.bitwise_or(mask1, mask2)

def laser_points_to_pixels(xyz, rotation, translation, fx, fy, cx, cy):
    """
    Convertir cada punto del lidar en el pixel (u, v) donde caería si la cámara lo estuviera mirando.

    Parámetros: xyz son los puntos del lidar (N, 3) en su frame. rotation es una matriz 3x3 y translation un vector de 3
    (la tf laser_link -> cam_1_color_optical_frame.
    La matriz sale de su cuaternión con scipy: Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()); fx, fy, cx, cy son
    los intrínsecos de la cámara (de camera_info).
    """
    with np.errstate(invalid='ignore'):  # rayos sin eco (inf): dan nan y después se descartan
        camera_points = xyz @ rotation.T + translation
    x_cam = camera_points[:, 0]
    y_cam = camera_points[:, 1]
    z_cam = camera_points[:, 2]

    in_front = z_cam > 0.01

    # proyeccion pinhole
    with np.errstate(divide='ignore', invalid='ignore'):
        u = fx * (x_cam / z_cam) + cx
        v = fy * (y_cam / z_cam) + cy

    return u, v, in_front

class VisionProcessor:
    def __init__(self, red_hue_low_1, red_hue_high_1, red_hue_low_2, red_hue_high_2, min_saturation, min_value, half_box, erosion_px, max_jump, min_rays):
        # Umbrales HSV, tamaño de la caja y filtros de paralaje: los valores están solo en params.yaml (sección perception), comentados.
        # Elegidos con fotos reales del robot con la luz alterada
        self.red_hue_low_1 = red_hue_low_1
        self.red_hue_high_1 = red_hue_high_1
        self.red_hue_low_2 = red_hue_low_2
        self.red_hue_high_2 = red_hue_high_2
        self.min_saturation = min_saturation
        self.min_value = min_value
        self.half_box = half_box  # m: de la cara que ve el láser al centro, que es lo que mide el juez

        # Contra el paralaje en los bordes de la caja
        self.erosion_px = int(erosion_px)  # píxeles que se le sacan a la silueta roja de cada lado
        self.max_jump = max_jump  # m: un salto mayor entre rayos rojos vecinos corta el tramo
        self.min_rays = int(min_rays)  # tramos más cortos se descartan

    def extract_box_centers(self, bgr_image, ranges, angles, rotation, translation, fx, fy, cx, cy):
        """
        Toma la imagen y el láser, devuelve una lista de puntos (x_local, y_local) indicando dónde hay cajas rojas.
        Quedan en el frame del lidar (laser_link), que está girado 180° respecto del robot: falta llevarlos a map con
        TF.

        Paralaje: un rayo que pasa rozando la caja pega en la pared, pero en la foto cae 1-2 px adentro del borde rojo
        (cámara y lidar en lugares distintos, bordes difusos) y daría una caja falsa metida en la pared.

        Dos filtros: se erosiona la silueta roja y los rayos rojos se cortan en tramos por saltos de distancia (una cara
        es una tira continua, un intruso, un rayo suelto que salta decenas de cm).

        En escenas sintéticas, las escenas con detecciones falsas bajan de 58 % a 4 % con pared detrás y de 48 % a 0 %
        en el borde de la foto (test_detector.py).
        """
        xyz = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)])  # Polares -> Cartesianas

        # 1. Armar máscara roja
        img_hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        red_mask = mask_red(
            img_hsv, self.red_hue_low_1, self.red_hue_high_1,
            self.red_hue_low_2, self.red_hue_high_2,
            self.min_saturation, self.min_value
        )
        kernel_side = 2 * self.erosion_px + 1
        red_mask = cv2.erode(red_mask, np.ones((kernel_side, kernel_side), np.uint8))  # el borde es donde foto y lidar no coinciden

        # 2. Proyectar el láser en la imagen
        u, v, in_front = laser_points_to_pixels(xyz, rotation, translation, fx, fy, cx, cy)

        # 3. Filtrar los rayos válidos que caen dentro de la foto
        height, width = bgr_image.shape[:2]
        valid = np.isfinite(ranges) & (ranges > 0.1)  # Filtro básico de validez
        in_image = in_front & valid & (u >= 0) & (u < width) & (v >= 0) & (v < height)

        # 4. Encontrar qué rayos pegaron en píxeles rojos
        idx_img = np.where(in_image)[0]
        v_val = v[idx_img].astype(int)
        u_val = u[idx_img].astype(int)

        # Filtramos de esos índices, cuáles cayeron en la máscara > 0
        is_red = red_mask[v_val, u_val] > 0
        red_idx = idx_img[is_red]  # Índices finales de los rayos del láser que tocaron la caja

        # Tramos de rayos rojos con distancia continua: los no rojos y los saltos cortan, los tramos cortos se descartan
        r = np.full(len(ranges), np.nan)
        r[red_idx] = ranges[red_idx]
        cut = int(np.argmax(np.isnan(r)))  # el scan arranca en un rayo no rojo: una caja justo en la costura no queda partida en dos
        spans = segments(np.roll(r, -cut), self.max_jump, self.min_rays)
        red_idx = (np.concatenate(spans).astype(int) + cut) % len(r) if spans else np.array([], int)

        # 5. CONVERSIÓN GEOMÉTRICA
        # El láser rebota en la cara de la caja y el juez mide desde el centro: se estira cada punto
        # media caja más a lo largo de su rayo, (r + half_box) / r.
        r = ranges[red_idx]
        centers = xyz[red_idx, :2] * ((r + self.half_box) / r)[:, None]

        return [tuple(p) for p in centers.tolist()]
