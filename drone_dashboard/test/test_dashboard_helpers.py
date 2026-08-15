import math

from sensor_msgs.msg import Image

from drone_dashboard.dashboard_node import DashboardNode


def test_decode_rgb_image_to_opencv_bgr():
    message = Image()
    message.height = 1
    message.width = 2
    message.encoding = "rgb8"
    message.step = 6
    message.data = [255, 0, 0, 0, 255, 0]

    image = DashboardNode._decode_image(message)

    assert image.shape == (1, 2, 3)
    assert image[0, 0].tolist() == [0, 0, 255]
    assert image[0, 1].tolist() == [0, 255, 0]


def test_non_finite_ros_values_become_json_null_values():
    assert DashboardNode._finite(12.5) == 12.5
    assert DashboardNode._finite(math.nan) is None
    assert DashboardNode._finite(math.inf) is None
