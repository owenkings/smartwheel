import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import wheelchair_sensors.camera_adapter_node as camera_module  # noqa: E402
from wheelchair_sensors.camera_adapter_node import (  # noqa: E402
    CameraAdapter,
    CameraConfig,
    GStreamerMjpegAdapter,
    cv_frame_to_compressed,
    gstreamer_mjpeg_pipeline,
    jpeg_bytes_to_compressed,
    jpeg_bytes_to_cv_frame,
    rotate_frame,
)


def test_rotate_frame_180_preserves_shape_and_flips_pixels():
    frame = np.arange(18, dtype=np.uint8).reshape((2, 3, 3))

    rotated = rotate_frame(frame, 180)

    assert rotated.shape == frame.shape
    assert np.array_equal(rotated, frame[::-1, ::-1])


class ClosedCapture:
    def set(self, *_args):
        return True

    def isOpened(self):
        return False

    def release(self):
        return None


class FakeCv2:
    CAP_V4L2 = 200
    CAP_PROP_FOURCC = 6
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5
    CAP_PROP_BUFFERSIZE = 38

    def __init__(self):
        self.open_calls = 0

    @staticmethod
    def VideoWriter_fourcc(*_args):
        return 0

    def VideoCapture(self, *_args):
        self.open_calls += 1
        return ClosedCapture()


def test_missing_camera_open_is_rate_limited(monkeypatch):
    fake_cv2 = FakeCv2()
    now = [10.0]
    monkeypatch.setattr(camera_module, "cv2", fake_cv2)
    monkeypatch.setattr(camera_module.time, "monotonic", lambda: now[0])
    adapter = CameraAdapter(
        CameraConfig("missing", "/camera/missing", "missing_link", "/dev/video9"),
        width=640,
        height=480,
        fps=30.0,
        reconnect_interval_sec=3.0,
    )

    assert adapter.read_image() is None
    assert fake_cv2.open_calls == 1

    now[0] = 11.0
    assert adapter.read_image() is None
    assert fake_cv2.open_calls == 1

    now[0] = 13.1
    assert adapter.read_image() is None
    assert fake_cv2.open_calls == 2


def test_compressed_frame_is_valid_jpeg_and_clamps_quality():
    if camera_module.CompressedImage is None:
        return
    frame = np.full((12, 16, 3), 127, dtype=np.uint8)
    stamp = camera_module.CompressedImage().header.stamp
    stamp.sec = 1
    stamp.nanosec = 2

    message = cv_frame_to_compressed(frame, stamp, "camera_test", 200)

    assert message.header.frame_id == "camera_test"
    assert message.format == "jpeg"
    assert bytes(message.data[:2]) == b"\xff\xd8"
    assert bytes(message.data[-2:]) == b"\xff\xd9"


def test_passthrough_message_preserves_camera_jpeg_bytes():
    if camera_module.CompressedImage is None:
        return
    source = np.full((12, 16, 3), 91, dtype=np.uint8)
    ok, encoded = camera_module.cv2.imencode(".jpg", source)
    assert ok
    payload = encoded.tobytes()
    stamp = camera_module.CompressedImage().header.stamp
    stamp.sec = 3
    stamp.nanosec = 4

    message = jpeg_bytes_to_compressed(payload, stamp, "camera_native")
    decoded = jpeg_bytes_to_cv_frame(bytes(message.data))

    assert message.header.frame_id == "camera_native"
    assert message.format == "jpeg"
    assert bytes(message.data) == payload
    assert decoded.shape == source.shape


def test_gstreamer_pipeline_requests_mjpeg_and_single_latest_buffer():
    pipeline = gstreamer_mjpeg_pipeline(
        "/dev/v4l/by-path/test", 640, 480, 30.0, "smartwheel_test_jpeg"
    )

    assert 'device="/dev/v4l/by-path/test"' in pipeline
    assert "image/jpeg,width=640,height=480,framerate=30/1" in pipeline
    assert "appsink name=smartwheel_test_jpeg" in pipeline
    assert "max-buffers=1 drop=true" in pipeline

    repaired = gstreamer_mjpeg_pipeline(
        "/dev/video-test",
        640,
        480,
        30.0,
        "repair_test",
        repair_jpeg=True,
        jpeg_quality=70,
    )
    assert "jpegdec ! jpegenc quality=70 !" in repaired


def test_gstreamer_adapter_running_requires_pipeline_and_live_capture_thread():
    camera = CameraConfig("front", "/camera/front/image_raw", "camera_front_link", "0")
    adapter = GStreamerMjpegAdapter(camera, 640, 480, 30.0)

    assert adapter.is_running() is False

    adapter.pipeline = object()
    adapter._capture_thread = type("Thread", (), {"is_alive": lambda self: True})()
    assert adapter.is_running() is True


def test_gstreamer_adapter_frame_callback_can_be_replaced_and_disabled():
    camera = CameraConfig("front", "/camera/front/image_raw", "camera_front_link", "0")
    adapter = GStreamerMjpegAdapter(camera, 640, 480, 30.0)
    first = lambda _payload: None
    second = lambda _payload: None

    adapter.set_frame_callback(first)
    assert adapter._frame_callback is first
    assert adapter.is_running() is False
    adapter.pipeline = object()
    adapter._capture_thread = type("Thread", (), {"is_alive": lambda self: True})()
    adapter._dispatch_thread = type("Thread", (), {"is_alive": lambda self: True})()
    assert adapter.is_running() is True
    adapter.set_frame_callback(second)
    assert adapter._frame_callback is second
    adapter.set_frame_callback(None)
    assert adapter._frame_callback is None
