"""Optional organized amplitude metadata; pixel indices never change XYZ order."""
import copy
import math

from .core import CalibrationError

MAX_ORGANIZED_POINTS = 20000


def validate_organized(value, rows_by_side):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {'left', 'right'}:
        raise CalibrationError('organized observations require explicit left/right metadata')
    result = {}
    for side in ('left', 'right'):
        item = value[side]
        if not isinstance(item, dict):
            raise CalibrationError('organized side must be an object')
        width, height = item.get('width'), item.get('height')
        if type(width) is not int or type(height) is not int or min(width, height) < 1 or \
                width * height > MAX_ORGANIZED_POINTS or item.get('order') != 'row_major':
            raise CalibrationError('organized requires bounded positive dimensions and row_major order')
        rows, amplitude = rows_by_side.get(side), item.get('amplitude')
        if not isinstance(rows, list) or len(rows) != width * height or \
                not isinstance(amplitude, list) or len(amplitude) != width * height:
            raise CalibrationError('organized dimensions, original XYZ rows and amplitude must match exactly')
        for number in amplitude:
            if number is None:
                continue
            try:
                valid = type(number) in (int, float) and math.isfinite(number) and number >= 0
            except OverflowError:
                valid = False
            if not valid:
                raise CalibrationError('amplitude values must be finite nonnegative numbers or null')
        result[side] = copy.deepcopy(item)
        result[side]['pixel_id_rule'] = 'row * width + column'
        result[side]['quantity'] = 'return_amplitude'
        result[side]['xyz_order_changed'] = False
    return result
