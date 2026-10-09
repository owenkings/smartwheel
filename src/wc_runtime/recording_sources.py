"""Read-only source inventory and explicit offline lidar selection, without ROS."""
from contextlib import closing
from pathlib import Path
import sqlite3


SIDES = ('left', 'right')
CLOUDS = ('raw', 'filtered')


def resolve_lidar_option(lidar=None, sides=None):
    """Retain the deployed --lidar API while accepting the panel's --sides."""
    for value in (lidar, sides):
        if value is not None and value not in ('all', *SIDES):
            raise ValueError('雷达来源必须是 all、left 或 right')
    if lidar is not None and sides is not None and lidar != sides:
        raise ValueError('--lidar 与 --sides 的雷达选择冲突，请保持一致或仅使用一个选项')
    return lidar if lidar is not None else sides


def source_inventory(bag):
    """Only a correctly typed topic containing messages establishes availability.

    A manifest's intended mode and an empty topic declaration do not establish
    that a sensor was recorded. Payload identity is checked by RecordedPackets
    during replay; this lightweight catalogue never deserializes cloud data.
    """
    files = sorted(Path(bag).glob('*.db3'))
    if not files:
        raise ValueError('录包没有 SQLite 数据文件')
    found = {side: set() for side in SIDES}
    topics = {'/wc_mapping/lidar_' + side + '/source_frame' +
              ('' if cloud == 'raw' else '_filtered'): (side, cloud)
              for side in SIDES for cloud in CLOUDS}
    try:
        for path in files:
            if path.is_symlink():
                raise ValueError('录包数据文件不能是符号链接')
            with closing(sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)) as db:
                db.execute('PRAGMA query_only=ON')
                for topic_id, name, kind in db.execute('SELECT id,name,type FROM topics'):
                    if name not in topics:
                        continue
                    if kind != 'wc_interfaces/msg/SourceFrame':
                        raise ValueError('雷达话题类型错误: ' + name)
                    if db.execute('SELECT 1 FROM messages WHERE topic_id=? LIMIT 1',
                                  (topic_id,)).fetchone() is not None:
                        side, cloud = topics[name]
                        found[side].add(cloud)
    except sqlite3.Error as error:
        raise ValueError('无法读取录包雷达来源: ' + str(error)) from error
    return {side: [cloud for cloud in CLOUDS if cloud in found[side]] for side in SIDES}


def select_sources(inventory, requested=None, *, cloud=None):
    """Normalize auto/default to actual available sources; explicit all needs both."""
    available = [side for side in SIDES if inventory.get(side)]
    if requested is None:
        requested = 'all' if len(available) == 2 else (available[0] if available else None)
    if requested not in ('all', *SIDES):
        raise ValueError('请选择录包中实际存在的雷达来源（双雷达、左雷达或右雷达）')
    selected = list(SIDES) if requested == 'all' else [requested]
    missing = [side for side in selected if side not in available]
    if missing:
        raise ValueError('录包没有所选雷达的点云: ' + '、'.join(missing))
    if cloud is not None:
        if cloud not in CLOUDS:
            raise ValueError('点云类型必须是 raw 或 filtered')
        missing = [side for side in selected if cloud not in inventory[side]]
        if missing:
            raise ValueError('所选雷达没有录制 ' + cloud + ' 点云: ' + '、'.join(missing))
    return requested, selected
