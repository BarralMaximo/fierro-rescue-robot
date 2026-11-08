"""El nodo de percepción sin sensores ni simulador: la cola de scans y el formato de los reportes."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from fierro.perception import build_report

PARAMS = Path(__file__).parents[1] / 'params' / 'params.yaml'

def test_node_keeps_scan_until_its_tf_and_photo_arrive():
    """
    perception con la TF o las fotos atrasadas (el software en otra compu): el scan queda pendiente hasta que llegan la
    TF de su instante y una foto posterior, y se procesa una sola vez; si esperó más de max_wait detrás del scan más
    nuevo, se descarta. Antes, con 0.14 s más de demora, no reportaba nada.
    """
    import rclpy
    import tf2_ros
    from rclpy.time import Time
    from sensor_msgs.msg import Image, LaserScan

    from fierro.perception import Perception

    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS), '-p', 'victims:=1'])
    node = None
    try:
        node = Perception()
        node.board, node.camera_info = object(), (1.0, 1.0, 0.0, 0.0)
        arrived, used = [False], []
        identity = SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=0.0, y=0.0, z=0.0), rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

        def lookup(target, source, t):
            if not arrived[0] and target in ('odom', 'map'): raise tf2_ros.LookupException('todavía no')
            return identity

        node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
        node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda image, desired_encoding: image)
        node.vision.extract_box_centers = lambda image, *a: used.append(Time.from_msg(image.header.stamp).nanoseconds * 1e-9) or []

        def scan(sec):
            m = LaserScan(angle_min=0.0, angle_increment=0.01, time_increment=0.0, ranges=[1.0] * 5)
            m.header.frame_id, m.header.stamp = 'laser', Time(seconds=sec).to_msg()
            return m

        def photo(sec):
            m = Image()
            m.header.frame_id, m.header.stamp = 'camera', Time(seconds=sec).to_msg()
            return m

        node.on_image(photo(0.98))
        node.on_scan(scan(1.0))
        assert used == [] and len(node.pending) == 1  # sin foto posterior: puede llegar una más cerca
        node.on_image(photo(1.01))
        assert used == [] and len(node.pending) == 1  # foto, pero sin la TF
        arrived[0] = True
        node.on_image(photo(1.04))
        assert used == [1.01] and node.pending == []  # la más cercana, una sola vez

        arrived[0] = False
        node.on_scan(scan(2.0))
        node.on_image(photo(2.02))
        node.on_scan(scan(2.0 + node.max_wait + 0.1))
        assert len(node.pending) == 1 and seconds_of(node.pending[0]) > 2.5  # el de los 2 s esperó demasiado: descartado
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()

def seconds_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def test_report_is_well_formed():
    msg = build_report(1.5, -2.0, Time(sec=3, nanosec=0))
    assert msg.header.frame_id == 'map'
    assert msg.point.x == pytest.approx(1.5)
    assert msg.point.y == pytest.approx(-2.0)

@pytest.mark.parametrize('x, y', [(math.nan, 0.0), (0.0, math.inf)])
def test_report_rejects_non_finite_values(x, y):
    with pytest.raises(ValueError):
        build_report(x, y, Time())
