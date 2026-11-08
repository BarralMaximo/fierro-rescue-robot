"""
Lo que comparten los tres nodos: la QoS del mapa, los parámetros, los tiempos de ROS y el giro de la IMU.

Así ningún nodo importa a otro: localizer, perception y navigator toman de acá lo común.
"""

import math

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def node_params(node):
    """
    Parámetros del nodo como diccionario (nombre: valor): los que ROS le cargó de params.yaml y del launch.

    Los nodos se crean con automatically_declare_parameters_from_overrides=True: ROS declara solo lo que trae el yaml,
    así que los valores están en un único lugar y el código no tiene copias. Si falta uno, el nodo falla al arrancar
    (KeyError con su nombre).
    """
    return {name: p.value for name, p in node.get_parameters_by_prefix('').items()}

def seconds(stamp):
    """
    Stamp de ROS (sec y nanosec, dos enteros). Lo convertimos en un solo float de segundos para poder restar e
    interpolar instantes.

    Se aplica siempre a header.stamp. Es decir, en el instante en que midió el sensor no la hora en que llegó el mensaje
    (un scan llega después de su stamp).
    Todos los stamps salen del mismo reloj (el robot, el simulador o el bag), así que se pueden comparar entre sí.
    """
    return Time.from_msg(stamp).nanoseconds * 1e-9

def yaw_of(q):
    """
    Yaw (rad, en (-π, π]) de un cuaternión de ROS: hacia dónde mira el robot visto desde arriba (tipo brújula).

    Un objeto en 3D gira de tres formas, como la cabeza:
        - yaw es decir "no" (girar a los costados)
        - pitch es decir "sí" (arriba y abajo)
        - roll es inclinarla hacia un hombro.

    El robot anda sobre un piso plano: pitch y roll quedan en 0 y el único giro que cambia es el yaw. ROS guarda las
    orientaciones como cuaterniones (x, y, z, w), que describen los tres giros juntos. Esta función separa el yaw.
    """
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def z_row(q):
    """
    Traduce el giro que mide la IMU al giro del robot. q es cómo está montada la IMU (la TF base_footprint -> imu_link).

    La IMU y el robot giran juntos y a la misma velocidad, pero cada uno lo cuenta con sus propios ejes. Un giro es
    positivo si, mirando desde la punta del eje z, se ve antihorario:
        - el robot tiene el z hacia el techo: doblar a la izquierda lo ve antihorario, +0.5 rad/s
        - la IMU de Donatello está boca abajo, con el z hacia el piso: el mismo giro lo ve horario, -0.5 rad/s

    Devuelve cuánto aporta cada eje de la IMU al giro del robot. En Donatello da (0, 0, -1): solo cuenta el z, con el
    signo cambiado.

    ACLARACIÓN: alcanzaría con multiplicar por -1 el z que mide la IMU, pero se calcula desde la TF por robustez: si la
    IMU del robot de la competencia llegara a estar montada distinto y su TF lo refleja, se ajustaría solo.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)])

def turn_source(d_odom, d_imu, imu_check):
    """
    De dónde sale el giro entre dos scans: 'imu', o 'odom' si la IMU no está (d_imu None: no llega, sin TF o
    desactivada) o gira al revés que las ruedas.

    La IMU es mejor (las ruedas patinan), pero no es indispensable: sin ella el filtro igual calza con el giro de /odom.
    Al revés significa montada distinto de lo que dice su TF; las ruedas patinan pero no giran para el otro lado. Giros
    de menos de imu_check (rad) no cuentan: con el robot derecho los dos son ruido.
    """
    if d_imu is None or (d_imu * d_odom < 0 and min(abs(d_imu), abs(d_odom)) > imu_check): return 'odom'

    return 'imu'

def interpolate(queue, t, max_age):
    """
    /odom y la IMU reportan cada ~0.1 s, nunca justo en t: se traza una recta entre el dato de antes y el de después y
    se lee el punto en t.
    Ej.: x = 1.00 a los 10.00 s y x = 1.05 a los 10.10 s -> a los 10.06 s, x = 1.03.

    El localizer la llama con odoms -> [x, y, yaw] y con turns -> [yaw]; el navigator, con turns. Devuelve cada columna
    menos el tiempo.

    Casos borde:
        - None (el scan se descarta): no hay datos, t es más viejo que todo el historial, o el último dato tiene más de
          max_age (sensor caído).
        - t después del último dato (el siguiente todavía no llegó): se prolonga la recta de los dos últimos datos,
          suponiendo que el robot no realiza cambios bruscos de movimiento.
    """
    if not queue or t > queue[-1][0] + max_age or t < queue[0][0]: return None
    data = np.asarray(queue)

    if t > data[-1, 0] and len(data) > 1 and data[-1, 0] > data[-2, 0]:  # el siguiente dato no llegó
        a, b = data[-2], data[-1]
        return [float(v) for v in b[1:] + (b[1:] - a[1:]) * (t - b[0]) / (b[0] - a[0])]

    return [float(np.interp(t, data[:, 0], data[:, k])) for k in range(1, data.shape[1])]
