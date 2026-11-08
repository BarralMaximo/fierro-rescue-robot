import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener

from fierro.core import Board, normalize
from fierro.mission import Mission
from fierro.ros_utils import MAP_QOS, interpolate, node_params, turn_source, yaw_of, z_row

class Navigator(Node):
    """
    Le pasa a Mission la pose (TF map -> base_footprint del localizer), el lidar (en map y, para frenar, en el frame del
    robot: near) y las posiciones de los reportes, y publica /cmd_vel y /done.

    No se mueve antes de /map ni antes de que el localizer publique la largada (/jar/pose). Frena mientras falte
    /jar/pose o un scan nuevo (sensor_timeout).
    Después de /done no vuelve a publicar.
    """

    def __init__(self):
        super().__init__('navigator', automatically_declare_parameters_from_overrides=True)  # Registra el nodo en ROS con los parámetros de params.yaml
        p = node_params(self)  # Los valores están solo en params.yaml (sección navigator), comentados. Mission los lee del nodo

        # Movimiento
        self.max_speed = p['max_speed']
        self.max_turn = p['max_turn']
        self.turn_lead = p['turn_lead']
        self.turn_gain = p['turn_gain']
        self.speed_gain = p['speed_gain']
        self.min_speed = p['min_speed']

        # Cuerpo y camino
        self.body_radius = p['body_radius']
        self.robot_radius = p['robot_radius']
        self.wheel_x = p['wheel_x']
        self.wheel_y = p['wheel_y']
        self.self_radius = p['self_radius']
        self.drivable_margin = p['drivable_margin']
        self.wall_margin = p['wall_margin']
        self.wall_penalty = p['wall_penalty']
        self.close_penalty = p['close_penalty']
        self.lookahead = p['lookahead']
        self.lookahead_short = p['lookahead_short']
        self.lookahead_min = p['lookahead_min']
        self.body_front = p['body_front']
        self.body_rear = p['body_rear']
        self.body_side = p['body_side']
        self.narrow_side = p['narrow_side']
        self.narrow_penalty = p['narrow_penalty']
        self.narrow_ahead = p['narrow_ahead']
        self.narrow_align = p['narrow_align']
        self.narrow_speed = p['narrow_speed']
        self.narrow_margin = p['narrow_margin']
        self.narrow_blocked = p['narrow_blocked']
        self.narrow_line = p['narrow_line']
        self.narrow_center = p['narrow_center']

        # Frenado, obstáculos y destrabe
        self.wall_tolerance = p['wall_tolerance']
        self.stop_time = p['stop_time']
        self.stop_margin = p['stop_margin']
        self.wall_stop = p['wall_stop']
        self.obstacle_range = p['obstacle_range']
        self.obstacle_scans = int(p['obstacle_scans'])
        self.clear_cooldown = p['clear_cooldown']
        self.max_blocked = p['max_blocked']
        self.retreat_radius = p['retreat_radius']
        self.retreat_speed = p['retreat_speed']

        # Guarda de la cola y motores
        self.tail_start = p['tail_start']
        self.guard_horizon = p['guard_horizon']
        self.guard_margin = p['guard_margin']
        self.guard_stall = p['guard_stall']
        self.guard_release = p['guard_release']
        self.slow_time = p['slow_time']
        self.slow_min = p['slow_min']
        self.turn_clearance = p['turn_clearance']
        self.motor_min_speed = p['motor_min_speed']
        self.motor_min_step = p['motor_min_step']
        self.motor_min_turn = p['motor_min_turn']
        self.motor_turn_tolerance = p['motor_turn_tolerance']

        # Recorrido
        self.lidar_range = p['lidar_range']
        self.tour_step = p['tour_step']
        self.tour_coverage = p['tour_coverage']
        self.goal_tolerance = p['goal_tolerance']
        self.pending_fraction = p['pending_fraction']
        self.pending_min = int(p['pending_min'])
        self.seen_margin = p['seen_margin']

        # Candidatos
        self.detect_jump = p['detect_jump']
        self.detect_tolerance = p['detect_tolerance']
        self.detect_points = int(p['detect_points'])
        self.detect_iterations = int(p['detect_iterations'])
        self.face_min = p['face_min']
        self.face_max = p['face_max']
        self.candidate_scans = int(p['candidate_scans'])
        self.detect_match = p['detect_match']
        self.detect_keep = p['detect_keep']
        self.glimpse_scans = int(p['glimpse_scans'])
        self.glimpse_size = p['glimpse_size']
        self.candidate_merge = p['candidate_merge']
        self.route_distance = p['route_distance']
        self.turning_rate = p['turning_rate']
        self.confirm_range = p['confirm_range']
        self.confirm_radius = p['confirm_radius']
        self.confirm_misses = int(p['confirm_misses'])
        self.occlusion_margin = p['occlusion_margin']

        # Cámara y percepción
        self.camera_range = p['camera_range']
        self.camera_fov = p['camera_fov']
        self.view_fraction = p['view_fraction']
        self.approach = p['approach']
        self.look_range = p['look_range']
        self.look_margin = p['look_margin']
        self.match_radius = p['match_radius']
        self.clear_time = p['clear_time']
        self.watch_time = p['watch_time']
        self.watch_baseline = p['watch_baseline']
        self.watch_still = p['watch_still']

        # Regreso
        self.time_limit = p['time_limit']
        self.return_pace = p['return_pace']
        self.return_margin = p['return_margin']
        self.start_tolerance = p['start_tolerance']
        self.return_max_blocked = p['return_max_blocked']

        # Nodo
        self.base_frame = p['base_frame']
        self.rate_hz = p['rate_hz']
        self.sensor_timeout = p['sensor_timeout']
        self.start_match = p['start_match']
        self.victims = int(p['victims'])  # N: llega del launch (victims:=N)
        self.imu_check = p['imu_check']
        self.imu_gap = p['imu_gap']
        self.max_age = p['max_age']

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Empty, '/done', 10)

        # Escuchamos
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/jar/pose', self.on_pose, 10)
        self.create_subscription(PoseArray, '/jar/red', self.on_red, 10)
        self.create_subscription(PointStamped, '/victimas', self.on_victim, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_timer(1.0 / self.rate_hz, self.control)

        # Estado inicial
        self.board = self.mission = self.t0 = self.start = self.ready_at = None
        self.finished, self.reports = False, []  # posiciones de lo publicado en /victimas (por perception)
        self.lidar = np.zeros((0, 2))
        self.scan_t = 0  # ns, hora de los sensores del último scan: la nuestra puede ser otra (el software en otra compu)
        self.pose_at = self.lidar_at = None  # cuándo llegó la última /jar/pose y el último scan pasado a map (nuestra hora)
        self.red = []  # dónde ve rojo la percepción sin haberlo reportado (/jar/red)
        self.pending = None  # scan cuya TF todavía no llegó: se reintenta en cada ciclo (en el robot real llega ~20 ms después)
        self.scan_base = None  # el último scan para near: (t, pose de las ruedas en t (odom), puntos en base_footprint)
        self.turns = deque(maxlen=400)  # (t, giro integrado del giróscopo)
        self.imu_yaw, self.imu_t, self.imu_row = 0.0, None, None

    # ---------- entradas ----------
    def on_map(self, msg):
        """Guarda el mapa una sola vez. Su llegada es la largada: el tiempo de la misión se cuenta desde acá."""
        if self.board is None: self.board, self.t0 = Board.from_msg(msg), self.get_clock().now()

    def on_pose(self, msg):
        """
        La primera pose del localizer es la largada (adonde vuelve). No sale de la TF: map -> odom va adelantada tf_lead
        (0.3 s) y en ese lapso la TF todavía da la pose de antes de localizarse (con el robot corrido 18 cm del origen,
        volvía al origen).

        El localizer publica una con cada scan: si dejan de llegar, control frena.
        """
        if self.start is None: self.start = (msg.pose.position.x, msg.pose.position.y)
        self.pose_at = self.get_clock().now()

    def on_red(self, msg):
        self.red = [(p.position.x, p.position.y) for p in msg.poses]

    def on_victim(self, msg):
        self.reports.append((msg.point.x, msg.point.y))

    def on_imu(self, msg):
        """
        Integra el giróscopo (t, giro) para near, como localizer.on_imu. Sin sesgo: en los ~0.1 s entre el scan y el
        control, 0.02 rad/s son 0.1°.
        """
        if self.imu_row is None:
            try: self.imu_row = z_row(self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time()).transform.rotation)
            except Exception: return

        w, t = msg.angular_velocity, Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.imu_t is not None and 0.0 < t - self.imu_t < self.imu_gap: self.imu_yaw += float(self.imu_row @ [w.x, w.y, w.z]) * (t - self.imu_t)
        self.imu_t = t
        self.turns.append((t, self.imu_yaw))

    def on_scan(self, msg):
        """
        Guarda los puntos del scan en el frame map (obstáculos, candidatos) y en base_footprint (near, para frenar), con
        la TF de su instante (use_scan).
        Si todavía no llegó, queda pendiente y se reintenta en cada ciclo de control: en el bag, al llegar el scan falta
        la TF de odom en el 35 % de los casos y llega 18 ms después (mediana; máx. 39). Descartarlos dejaba hasta 0.4 s
        sin lidar nuevo. Antes de reemplazarlo, el pendiente tiene otra oportunidad.
        """
        self.use_scan()
        self.pending = msg
        self.use_scan()

    def use_scan(self):
        """
        Pasa el scan pendiente al frame map con la TF del instante del scan, no con la última: girando, la diferencia
        corre las paredes e inventa obstáculos.
        También lo guarda en base_footprint con la pose de las ruedas en ese instante (scan_base, para near). Si la TF
        todavía no está, sigue pendiente.
        """
        msg = self.pending
        if msg is None: return

        t = Time.from_msg(msg.header.stamp) + Duration(seconds=msg.time_increment * (len(msg.ranges) - 1) / 2)
        self.scan_t = t.nanoseconds
        if self.ready_at is None or self.scan_t <= self.ready_at:
            # antes, la TF todavía no tenía la largada: las paredes salían corridas y parecían objetos
            self.pending = None
            return

        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, t).transform
            ob = self.tf.lookup_transform('odom', self.base_frame, t).transform
            mount = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time()).transform  # dónde está el lidar en el robot: fijo
        except Exception: return

        self.pending = None

        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > self.self_radius) & (r < self.lidar_range)
        th = yaw_of(tf.rotation)
        c, s = math.cos(th), math.sin(th)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        self.lidar = np.column_stack([tf.translation.x + c * x - s * y, tf.translation.y + s * x + c * y])
        c, s = math.cos(yaw_of(mount.rotation)), math.sin(yaw_of(mount.rotation))
        base = np.column_stack([mount.translation.x + c * x - s * y, mount.translation.y + s * x + c * y])  # mismos puntos y orden que lidar (is_wall)
        self.scan_base = (t.nanoseconds * 1e-9, (ob.translation.x, ob.translation.y, yaw_of(ob.rotation)), base)
        self.lidar_at = self.get_clock().now()

    # ---------- control ----------
    def near(self):
        """
        El último scan en el frame del robot ahora, para frenar (_clear_ahead, tail_barrier, slow_down, giro_lidar):
        corrido lo que se movió el robot desde el scan, con la traslación de las ruedas (odom) y el giro del giróscopo,
        que no patina. Si la IMU no llega o gira al revés, el giro de las ruedas (turn_source). No pasa por map: una
        corrección del localizer en el medio no mueve lo que tiene cerca. En el bag, llevando cada scan al siguiente,
        calza 0.90 cm (p95; 0.62 con la escala del encoder corregida) contra 1.22 pasando por map. None si todavía no
        hay scan o falta la TF (Mission usa la pose).
        """
        if self.scan_base is None: return None

        t0, (x0, y0, th0), pts = self.scan_base
        try: tf = self.tf.lookup_transform('odom', self.base_frame, Time())
        except Exception: return None

        t1, x1, y1, th1 = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9, tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)
        a, b = interpolate(self.turns, t0, self.max_age), interpolate(self.turns, t1, self.max_age)
        d_odom, d_imu = normalize(th1 - th0), None if a is None or b is None else b[0] - a[0]
        dth = d_imu if turn_source(d_odom, d_imu, self.imu_check) == 'imu' else d_odom
        c, s = math.cos(th0), math.sin(th0)
        dx, dy = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)  # lo que avanzó, en el frame del robot en el scan
        c, s = math.cos(dth), math.sin(dth)

        return (pts - (dx, dy)) @ np.array([[c, -s], [s, c]])

    def pose(self):
        """Pose (x, y, yaw) del robot en map con la última TF. None si todavía no está."""
        try: tf = self.tf.lookup_transform('map', self.base_frame, Time()).transform
        except Exception: return None

        return tf.translation.x, tf.translation.y, yaw_of(tf.rotation)

    def control(self):
        """
        Un ciclo de control (rate_hz): pose -> Mission.step -> /cmd_vel, y /done al terminar.

        La Mission se crea con la primera pose localizada, que es la largada. Si algo falla, frena y sigue intentando:
        un crash es la corrida perdida.
        """
        if self.board is None or self.finished: return  # antes del mapa no se mueve; después de /done, tampoco

        try:
            self.use_scan()  # el scan que esperaba su TF
            pose = self.pose()
            if pose is None or self.start is None: return

            if self.mission is None:
                if math.hypot(pose[0] - self.start[0], pose[1] - self.start[1]) > self.start_match: return  # la TF todavía no tiene la largada del localizer
                self.mission, self.ready_at = Mission(self.board, self.start, self), self.scan_t  # los scans que siguen ya tienen la largada en la TF
                self.get_logger().info(f'recorrido: {len(self.mission.points)} puntos de paso desde ({pose[0]:.2f}, {pose[1]:.2f})')

            # Sin /jar/pose o sin scan nuevo, frena: se cayó el localizer o el lidar, y la TF sigue con la odometría sola (y el lidar, congelado)
            now = self.get_clock().now()
            lost = [name for name, at in (('/jar/pose', self.pose_at), ('scan', self.lidar_at)) if at is None or (now - at).nanoseconds * 1e-9 > self.sensor_timeout]
            if lost:
                self.cmd_pub.publish(Twist())
                self.mission.t_prev = self.mission.moved = None  # al volver, ese rato no cuenta (dt = 0): ni mirando un candidato, ni tapado, ni sin moverse
                if self.lidar_at is not None:
                    # al arrancar, hasta el primer scan después de la largada, frena sin avisar
                    self.get_logger().warn(f'freno: hace más de {self.sensor_timeout:.1f} s que no llega {" ni ".join(lost)}', throttle_duration_sec=2.0)
                return

            t = (now - self.t0).nanoseconds * 1e-9
            state = self.mission.state
            (vx, vy, w), done = self.mission.step(*pose, t, self.lidar, self.reports, self.victims, self.red, self.near())

            # Log: cambios de estado, candidatos, puntos salteados y qué lo frenó
            if self.mission.state != state: self.get_logger().info(f'{state} -> {self.mission.state} (punto {self.mission.k}, t={t:.0f} s)')
            while self.mission.log:
                self.get_logger().info(f'{self.mission.log.pop(0)} (t={t:.0f} s)')
            if self.mission.blocked_since is not None and self.mission.blocker is not None:
                tx, ty = self.mission.blocker
                c, s = math.cos(pose[2]), math.sin(pose[2])
                mx, my = pose[0] + c * tx - s * ty, pose[1] + s * tx + c * ty
                self.get_logger().warn(
                    f'paso tapado en ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f}°): lidar a ({tx:.2f}, {ty:.2f}) del robot = '
                    f'({mx:.2f}, {my:.2f}) del mapa, a {float(self.board.distance(mx, my)):.2f} m de una pared del mapa; desde t={self.mission.blocked_since:.0f} s',
                    throttle_duration_sec=1.0)

            if done:  # primero /done, con las ruedas adentro; después el freno
                self.done_pub.publish(Empty())
                self.finished = True
                self.get_logger().info(f'/done en ({pose[0]:.2f}, {pose[1]:.2f}), t={t:.0f} s')

            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = float(vx), float(vy), float(w)
            self.cmd_pub.publish(cmd)
        except Exception as e:
            self.cmd_pub.publish(Twist())
            self.get_logger().error(f'control: {e!r}', throttle_duration_sec=1.0)

def main(args=None):
    """
    Punto de entrada del ejecutable navigator (setup.py). Lo arranca fierro.launch.py con params.yaml.

    rclpy.spin alcanza: un solo timer y callbacks cortos. Ctrl+C y el apagado del launch terminan sin traceback.
    """
    rclpy.init(args=args)
    node = Navigator()

    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    except Exception:
        if rclpy.ok(): raise  # con ROS ya apagado (Ctrl+C, el launch), spin puede tirar RCLError al armar la espera: no es un error
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
