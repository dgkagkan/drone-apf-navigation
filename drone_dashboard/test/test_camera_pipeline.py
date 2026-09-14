"""Check camera concurrency, recovery and snapshot isolation without a ROS graph."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from drone_dashboard.dashboard_node import DashboardNode


def camera_node():
    return SimpleNamespace(
        _lock=threading.Lock(),
        _camera_lock=threading.Lock(),
        _camera_frames={},
        _last_camera_encode={},
        _camera_encoding=set(),
        _camera_rate_hz=30.0,
        _camera_resolution="640x360",
        _jpeg_quality=72,
        _decode_image=DashboardNode._decode_image,
        get_logger=Mock(),
    )


def camera_message():
    message = Image(height=1, width=1, encoding="rgb8", step=3)
    message.data = [255, 0, 0]
    return message


def test_camera_parameter_callback_reads_rclpy_parameter_values():
    node = camera_node()

    result = DashboardNode._on_parameters_set(node, [
        Parameter("camera_rate_hz", value=15.0),
        Parameter("camera_resolution", value="320x180"),
        Parameter("jpeg_quality", value=80),
    ])

    assert result.successful is True
    assert node._camera_rate_hz == 15.0
    assert node._camera_resolution == "320x180"
    assert node._jpeg_quality == 80


def test_slow_encode_drops_overlapping_callbacks(monkeypatch):
    node = camera_node()
    entered = threading.Event()
    release = threading.Event()

    def slow_encode(*args):
        entered.set()
        assert release.wait(2)
        return True, np.array([1, 2, 3], dtype=np.uint8)

    encoder = Mock(side_effect=slow_encode)
    monkeypatch.setattr("drone_dashboard.dashboard_node.cv2.imencode", encoder)
    worker = threading.Thread(target=DashboardNode._on_camera, args=(node, "drone_1", camera_message()))
    worker.start()
    try:
        assert entered.wait(2)
        # Simulate another callback after the rate interval but before encoding ends.
        node._last_camera_encode["drone_1"] = 0.0
        DashboardNode._on_camera(node, "drone_1", camera_message())
        assert encoder.call_count == 1
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert not node._camera_encoding
    assert DashboardNode.camera_frame(node, "drone_1") == b"\x01\x02\x03"


def test_encode_failure_releases_camera_for_next_frame():
    node = camera_node()
    bad_image = camera_message()
    bad_image.encoding = "unsupported"
    DashboardNode._on_camera(node, "drone_1", bad_image)
    assert not node._camera_encoding
    assert DashboardNode.camera_frame(node, "drone_1") is None
    node._last_camera_encode["drone_1"] = 0.0
    DashboardNode._on_camera(node, "drone_1", camera_message())
    assert DashboardNode.camera_frame(node, "drone_1").startswith(b"\xff\xd8")


def test_stale_camera_expires_and_recovers():
    node = camera_node()
    node._camera_frames["drone_1"] = (time.monotonic() - 10.0, b"old")
    assert DashboardNode.camera_frame(node, "drone_1") is None
    DashboardNode._on_camera(node, "drone_1", camera_message())
    assert DashboardNode.camera_frame(node, "drone_1") is not None


def test_snapshot_survives_callback_replacements():
    node = camera_node()
    node._state = {"connected": True}
    node._telemetry = {"drone_1": {"value": 1}}
    node._motion = {"drone_1": {"speed_m_s": 1}}
    node._paths = {"drone_1": {"flown": [{"x": 1}]}}
    node._camera_frames = {
        "drone_1": (time.monotonic(), b"live"),
        "drone_2": (time.monotonic() - 10.0, b"old"),
    }
    snapshot = DashboardNode.snapshot(node)
    node._state = {"connected": False}
    node._telemetry["drone_1"] = {"value": 2}
    node._motion["drone_1"] = {"speed_m_s": 2}
    node._paths["drone_1"]["flown"] = [{"x": 2}]
    assert snapshot["connected"] is True
    assert snapshot["telemetry"]["drone_1"]["value"] == 1
    assert snapshot["motion"]["drone_1"]["speed_m_s"] == 1
    assert snapshot["paths"]["drone_1"]["flown"] == [{"x": 1}]
    assert snapshot["camera_drones"] == ["drone_1"]


def test_camera_snapshot_is_saved_per_drone(tmp_path):
    node = camera_node()
    node._photo_save_dir = tmp_path
    node._camera_frames["drone_1"] = (time.monotonic(), b"jpeg-frame")

    saved_path = DashboardNode.save_camera_snapshot(node, "drone_1")

    assert saved_path.parent == tmp_path / "drone_1"
    assert saved_path.name.startswith("snapshot_")
    assert saved_path.suffix == ".jpg"
    assert saved_path.read_bytes() == b"jpeg-frame"


def test_camera_snapshot_rejects_path_traversal(tmp_path):
    node = camera_node()
    node._photo_save_dir = tmp_path
    node._camera_frames["drone_1"] = (time.monotonic(), b"jpeg-frame")

    try:
        DashboardNode.save_camera_snapshot(node, "../drone_1")
    except ValueError as exception:
        assert str(exception) == "invalid drone id"
    else:
        raise AssertionError("path traversal drone id was accepted")


def test_server_recording_is_saved_per_drone(tmp_path):
    node = camera_node()
    node._record_save_dir = tmp_path

    saved_path = DashboardNode.save_recording(node, "drone_2", b"webm-data")

    assert saved_path.parent == tmp_path / "drone_2"
    assert saved_path.name.startswith("recording_")
    assert saved_path.suffix == ".webm"
    assert saved_path.read_bytes() == b"webm-data"


def test_native_folder_picker_configures_selected_directory(tmp_path, monkeypatch):
    node = camera_node()
    picker_result = SimpleNamespace(
        returncode=0,
        stdout=str(tmp_path),
        stderr="",
    )
    monkeypatch.setattr(
        "drone_dashboard.dashboard_node.shutil.which",
        lambda name: "/usr/bin/zenity" if name == "zenity" else None,
    )
    picker = Mock(return_value=picker_result)
    monkeypatch.setattr("drone_dashboard.dashboard_node.subprocess.run", picker)

    selected_directory = DashboardNode.pick_media_directory(node, "photo")

    assert selected_directory == tmp_path
    assert node._photo_save_dir == tmp_path
    picker.assert_called_once()


def test_server_recording_writes_ros_camera_frames(tmp_path, monkeypatch):
    node = camera_node()
    node._recording_lock = threading.Lock()
    node._server_video_recordings = {}
    node._record_save_dir = tmp_path
    node._recording_rate_hz = 30.0
    node._camera_frames["drone_1"] = (time.monotonic(), b"jpeg-frame")
    writer = Mock()
    writer.isOpened.return_value = True
    monkeypatch.setattr("drone_dashboard.dashboard_node.cv2.VideoWriter", Mock(return_value=writer))
    monkeypatch.setattr("drone_dashboard.dashboard_node.cv2.VideoWriter_fourcc", Mock(return_value=1))

    DashboardNode.start_server_recording(node, "drone_1")
    DashboardNode._record_camera_frame(
        node,
        "drone_1",
        np.zeros((4, 6, 3), dtype=np.uint8),
        time.monotonic(),
    )
    recording = node._server_video_recordings["drone_1"]
    recording.temporary_path.write_bytes(b"avi-data")
    saved_path = DashboardNode.stop_server_recording(node, "drone_1")

    writer.write.assert_called_once()
    writer.release.assert_called_once()
    assert saved_path.suffix == ".avi"
    assert saved_path.read_bytes() == b"avi-data"
