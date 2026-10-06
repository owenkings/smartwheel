"""Offline dual lidar calibration. Live eligibility is explicit and evidence bound."""

from .core import (
    CalibrationError,
    QualityProfile,
    calibrate,
    fit_clock_model,
    fit_ground_plane,
    validate_calibration,
    validate_transform,
)

__version__ = "0.1.0"
__all__ = [
    "CalibrationError", "QualityProfile", "calibrate", "fit_clock_model",
    "fit_ground_plane", "validate_calibration", "validate_transform",
]
