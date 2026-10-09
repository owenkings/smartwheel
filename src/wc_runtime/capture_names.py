"""User-visible recording folder names, independent of runtime session IDs."""
from datetime import datetime
import re
import unicodedata

DEFAULT_NAMING = dict(prefix='capture', year=True, date=True, time=True)


def validate_folder_name(value):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError('数据名称不能为空，且不能以空格开头或结尾')
    if value.startswith('.') or value.endswith('.') or any(c in value for c in '/\\<>:"|?*'):
        raise ValueError('数据名称不能包含路径分隔符或 / \\ < > : " | ? *，也不能以点开头或结尾')
    if any(unicodedata.category(c).startswith('C') for c in value):
        raise ValueError('数据名称不能包含控制字符或不可见字符')
    if len(value.encode('utf-8')) > 220:
        raise ValueError('数据名称过长，请缩短前缀')
    if re.fullmatch(r'(?i)(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', value):
        raise ValueError('该名称是系统保留名称，请更换')
    return value


def naming_options(value=None):
    value = DEFAULT_NAMING if value is None else value
    if not isinstance(value, dict) or set(value) != set(DEFAULT_NAMING):
        raise ValueError('请选择数据前缀和年份、日期、时间后缀')
    if not isinstance(value['prefix'], str) or any(type(value[key]) is not bool for key in ('year','date','time')):
        raise ValueError('数据命名选项无效')
    result = dict(value)
    result['prefix'] = result['prefix'].strip().strip('_')
    if result['prefix']:
        validate_folder_name(result['prefix'])
    if not result['prefix'] and not any(result[key] for key in ('year','date','time')):
        raise ValueError('请输入数据前缀或至少选择一种时间后缀')
    return result


def recording_folder_name(value=None, instant=None):
    value = naming_options(value)
    instant = datetime.now() if instant is None else instant
    parts = [value['prefix']] if value['prefix'] else []
    for key, pattern in (('year','%Y'),('date','%m%d'),('time','%H%M%S')):
        if value[key]: parts.append(instant.strftime(pattern))
    return validate_folder_name('_'.join(parts))


def available_folder(root, name, active_paths=()):
    """Keep existing data intact; the recorder still creates with exist_ok=False."""
    from pathlib import Path
    root = Path(root)
    validate_folder_name(name)
    protected = {Path(path).absolute() for path in active_paths}
    for number in range(1, 10000):
        candidate = root / (name if number == 1 else name + '_{:02d}'.format(number))
        if not candidate.exists() and not candidate.is_symlink() and candidate.absolute() not in protected:
            validate_folder_name(candidate.name)
            return candidate
    raise ValueError('同名数据过多，请更换前缀')
