import math

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from fierro.core import Board, DetectionClusterer, normalize
from fierro.detector import VisionProcessor
from fierro.localizer import MAP_QOS, node_params, seconds, yaw_of

THRESHOLDS = ('red_hue_low_1', 'red_hue_high_1', 'red_hue_low_2', 'red_hue_high_2', 'min_saturation', 'min_value', 'half_box', 'erosion_px', 'max_jump', 'min_rays')

REPORT_FRAME = 'map'

def build_report(x, y, now):
    """
    PointStamped de un reporte, o ValueError. Cada mensaje en /victimas gasta uno de los N reportes, puntúe o no: nunca
    sale uno mal formado.

    x, y: centro de la caja en metros, en el frame map. ahora: builtin_interfaces/Time.
    """
    if not (math.isfinite(x) and math.isfinite(y)): raise ValueError(f'reporte con valores no finitos: x={x}, y={y}')
    msg = PointStamped()
    msg.header.frame_id = REPORT_FRAME
    msg.header.stamp = now
    msg.point.x = float(x)
    msg.point.y = float(y)
    msg.point.z = 0.0
    return msg

class Perception(Node):
    def __init__(self):
        super().__init__('perception', automatically_declare_parameters_from_overrides=True)
        p = node_params(self)  # Los valores están solo en params.yaml (sección perception), comentados

        # Cantidad de víctimas a buscar (N), que llega como argumento del launch (victims:=N): nunca se reporta más
        self.victims = int(p['victims'])

        # Detector: umbrales HSV del rojo, tamaño de la caja y filtros de paralaje
        self.vision = VisionProcessor(**{k: float(p[k]) for k in THRESHOLDS})
        self.deskew = p['deskew']  # orden temporal de los rayos, como en el localizer; 0: todos en el instante medio
        self.min_wall_distance = p['min_wall_distance']  # m: un rayo que pegó a esto o menos de una pared del mapa es pared, no caja (0: en una celda de pared)
        self.report_rays = int(p['report_rays'])  # rayos rojos sumados para reportar una caja
        self.min_observations = int(p['min_observations'])  # en al menos estos scans
        self.max_spread = p['max_spread']  # m: grupos más dispersos no se reportan
        self.red_observations = int(p['red_observations'])  # para avisarle al navegador que ahí hay rojo (/jar/red)
        self.max_photo_lag = p['max_photo_lag']  # s: sin foto tan cerca del scan, el scan no se usa
        self.max_wait = p['max_wait']  # s: un scan espera hasta esto a que lleguen la TF de su instante y su foto; después se descarta

        self.clusterer = DetectionClusterer(p['group_radius'])

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.victims_pub = self.create_publisher(PointStamped, '/victimas', 10)
        self.red_pub = self.create_publisher(PoseArray, '/jar/red', 10)  # interno: dónde ve rojo sin reportar todavía (para el navegador)

        # Escuchamos
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(CameraInfo, '/cam_1/color/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/cam_1/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        # Estado inicial
        self.board = self.camera_info = None
        self.pending = []  # scans esperando la TF de su instante o su foto
        self.photos = []  # las de los últimos max_wait + max_photo_lag s (por stamp): de acá sale la más cercana a cada scan
        self.reports = 0
        self.get_logger().info(f'Hay que encontrar {self.victims} víctimas. Esperando el mapa en /map...')

    # ---------- entradas ----------
    def on_map(self, msg):
        if self.board is not None: return

        self.board = Board.from_msg(msg)
        self.get_logger().info(f'Mapa recibido: {msg.info.width}x{msg.info.height} celdas, resolución {msg.info.resolution:.3f} m')

    def on_camera_info(self, msg):
        self.camera_info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy

    def on_image(self, msg):
        t = seconds(msg.header.stamp)
        self.photos = [f for f in self.photos if t - seconds(f.header.stamp) < self.max_wait + self.max_photo_lag] + [msg]  # se convierte a OpenCV solo la que se usa
        self.process_pending()

    def on_scan(self, msg):
        """
        Encola el scan y procesa los que ya se pueden. La TF de su último rayo y la foto llegan después que el scan (TF
        no extrapola), y con el software en otra compu, más tarde todavía. Antes se procesaba el scan anterior al llegar
        el siguiente y, si faltaba algo, se perdía: con la TF 0.14 s más atrasada, o las fotos 0.3 s, no se reportaba
        nada.
        """
        if self.board is None or self.camera_info is None: return

        self.pending.append(msg)
        self.process_pending()
        self.publish_red()

    def process_pending(self):
        """
        Procesa los scans pendientes que ya tienen la TF de su instante y su foto. Descarta los que quedaron más de
        max_wait detrás del más nuevo.
        """
        if not self.pending: return

        latest, remaining = seconds(self.pending[-1].header.stamp), []
        for scan in self.pending:
            try: ready = self.process(scan)
            except tf2_ros.TransformException: ready = False  # todavía no llegó la TF de su instante (o no hay pose: antes de localizarse)
            except Exception as e:
                self.get_logger().error(f'percepción: {e!r}', throttle_duration_sec=1.0)
                ready = True
            if not ready and latest - seconds(scan.header.stamp) < self.max_wait: remaining.append(scan)
        self.pending = remaining

    # ---------- percepción ----------
    def process(self, scan):
        """
        Detecciones de un scan -> map -> DetectionClusterer -> reportes. False si tiene que esperar una foto (todavía no
        llegó ninguna posterior al scan, y la más cercana puede ser esa); tf2_ros.TransformException si falta la TF de
        su instante.

        Usa la foto más cercana al instante medio del scan y lleva cada rayo a ese instante: le suma a su ángulo lo que
        giró el lidar entre que lo midió y la foto (en el robot real los rayos se reparten en 0.13 s; girando, foto y
        scan se desfasan unos grados). El avance en ese lapso (< 3 cm) no se corrige.
        """
        n, dt = len(scan.ranges), scan.time_increment
        t0 = Time.from_msg(scan.header.stamp)
        t_mid = t0 + Duration(seconds=dt * (n - 1) / 2)
        if not self.photos or Time.from_msg(self.photos[-1].header.stamp) < t_mid: return False  # puede llegar una foto más cerca
        photo = min(self.photos, key=lambda f: abs((Time.from_msg(f.header.stamp) - t_mid).nanoseconds))
        t_photo = Time.from_msg(photo.header.stamp)
        if abs((t_photo - t_mid).nanoseconds) * 1e-9 > self.max_photo_lag: return True  # sin foto cerca del scan

        # Giro del lidar en odom al empezar y terminar el scan y en la foto -> ángulo de cada rayo en el instante de la foto
        laser = scan.header.frame_id
        tf = lambda target, t: self.tf_buffer.lookup_transform(target, laser, t).transform
        a, b, c = (yaw_of(tf('odom', t).rotation) for t in (t0, t0 + Duration(seconds=dt * (n - 1)), t_photo))
        m = tf('map', t_photo)  # antes del detector: sin pose todavía, el reintento no procesa la foto
        k = np.arange(n)
        fraction = 0.5 + self.deskew * (k - (n - 1) / 2) / max(n - 1, 1)  # 0 al empezar el scan, 1 al terminar
        angles = scan.angle_min + k * scan.angle_increment + normalize(a + fraction * normalize(b - a) - c)

        # Detector: rayos rojos -> centros de caja en el frame del lidar
        cam = self.tf_buffer.lookup_transform(photo.header.frame_id, laser, Time()).transform  # montaje: fijo
        q, v = cam.rotation, cam.translation
        centers = self.vision.extract_box_centers(
            self.bridge.imgmsg_to_cv2(photo, desired_encoding='bgr8'), np.asarray(scan.ranges), angles,
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), np.array([v.x, v.y, v.z]), *self.camera_info)
        if not centers: return True

        # A map. El rayo pegó media caja antes del centro: si ahí hay una pared del mapa, era la pared (luz cálida, paralaje)
        th = yaw_of(m.rotation)
        rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        centers = np.asarray(centers)
        hits = centers * (1 - self.vision.half_box / np.hypot(*centers.T))[:, None]
        centers, hits = (pts @ rot.T + (m.translation.x, m.translation.y) for pts in (centers, hits))
        centers = centers[self.board.distance(hits[:, 0], hits[:, 1]) > self.min_wall_distance]

        # Una observación por caja y por scan (la mediana de sus rayos) con cuántos rayos tuvo: ready() pide rayos y scans
        in_scan = DetectionClusterer(self.clusterer.radius)
        for x, y in centers: in_scan.add(x, y)
        for g in in_scan.groups: self.clusterer.add(*in_scan.center(g), len(g['obs']))

        self.report_ready()

        return True

    def report_ready(self):
        """
        Reporta cada caja que ready() da por buena y no se reportó, mientras queden reportes. Cada reporte gasta uno de
        los N.
        """
        for g in self.clusterer.groups:
            if self.reports >= self.victims: return
            if g['reported'] or not self.clusterer.ready(g, self.report_rays, self.min_observations, self.max_spread): continue

            x, y = self.clusterer.center(g)
            self.victims_pub.publish(build_report(x, y, self.get_clock().now().to_msg()))
            g['reported'], self.reports = True, self.reports + 1
            self.get_logger().info(f'víctima {self.reports}/{self.victims} en ({x:.2f}, {y:.2f}): {len(g["obs"])} observaciones, '
                                   f'{sum(o[2] for o in g["obs"])} rayos rojos')

    def publish_red(self):
        """
        Publica en /jar/red los grupos sin reportar con red_observations observaciones o más: el navegador sigue mirando
        un candidato con rojo cerca hasta el reporte y descarta en ~1 s uno sin rojo (sin esto, esperaba 3.5 s en cada
        señuelo).
        """
        msg = PoseArray()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        for g in self.clusterer.groups:
            if g['reported'] or len(g['obs']) < self.red_observations: continue

            pose = Pose()
            pose.position.x, pose.position.y = self.clusterer.center(g)
            msg.poses.append(pose)
        self.red_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Perception()

    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    except Exception:
        if rclpy.ok(): raise  # con ROS ya apagado (Ctrl+C, el launch), spin puede tirar RCLError al armar la espera: no es un error
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
