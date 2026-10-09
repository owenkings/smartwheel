"""Read device-scoped motion declarations with their immutable evidence bytes."""
import hashlib
from pathlib import Path

from .prepare_picker_input import project_path, _json_loads
from .storage_policy import logical_path
from .live_motion_options import validate_live_calibration
from .mapping_planar import validate_confirmed_bias


def load_live_calibration_files(root, config):
    """Compose only when selected; a declaration never proves physical accuracy."""
    if not config.get('motion_correction'):
        return config
    if config.get('live_motion_calibration') is not None:
        raise ValueError('在线运动修正只能读取 live_motion_calibration_config 的独立声明及依据文件')
    if not config.get('live_motion_calibration_config'):
        raise ValueError('在线运动修正缺少设备校正声明，请在设备参数中导入并确认')
    if config.get('confirmed_gyro_bias') is None or not config.get('gyro_bias_provenance'):
        raise ValueError('在线运动修正要求 gyro_bias_config 及其已核对 SHA-256 的独立依据')
    path = project_path(Path(root), config['live_motion_calibration_config'])
    if not path.is_file() or path.stat().st_size > 32768:
        raise ValueError('设备校正声明不存在或超过 32 KiB')
    raw = path.read_bytes()
    value = validate_live_calibration(_json_loads(raw.decode('utf-8')), config)
    confirmed = validate_confirmed_bias(config['confirmed_gyro_bias'], expected_sensor_id=config['imu_sensor_id'])
    if value['confirmed_gyro_bias'] != confirmed:
        raise ValueError('设备校正声明中的零偏与独立 gyro_bias_config 不一致')
    evidence_path = project_path(Path(root), path.with_name(path.stem+'.evidence.json'))
    if not evidence_path.is_file() or evidence_path.stat().st_size > 262144:
        raise ValueError('设备校正声明必须附带同名 .evidence.json 依据文件')
    evidence = evidence_path.read_bytes()
    if hashlib.sha256(evidence).hexdigest() != value['evidence_sha256']:
        raise ValueError('设备校正依据 SHA-256 与声明不一致')
    _json_loads(evidence.decode('utf-8'))
    config.update(live_motion_calibration=value,
        _live_motion_raw_text=raw.decode('utf-8'),
        _live_motion_evidence_raw_text=evidence.decode('utf-8'),
        live_motion_provenance=dict(source_path=logical_path(root, path),
            sha256=hashlib.sha256(raw).hexdigest(), snapshot_file='live_motion_calibration.json',
            evidence_sha256=hashlib.sha256(evidence).hexdigest(),
            evidence_snapshot_file='live_motion_calibration.evidence.json',
            physical_accuracy_validated=False, scope='EXPLICIT_DEVICE_DECLARATION'))
    return config
