"""Offline sensor parsing and metadata; this package never opens hardware."""

from .h30 import H30Parser, H30Frame, h30_checksum
from .metadata import ClockModel, SourceFrameBuilder, normalize_time_parts, timestamp_ns
from .pointcloud import copy_sdk_points, decode_pointcloud2, validate_pointcloud2
from .preflight import SensorEndpoint, validate_endpoints

__all__ = [
    "H30Parser", "H30Frame", "h30_checksum", "ClockModel", "SourceFrameBuilder",
    "normalize_time_parts", "timestamp_ns", "copy_sdk_points", "decode_pointcloud2",
    "validate_pointcloud2", "SensorEndpoint", "validate_endpoints",
]
