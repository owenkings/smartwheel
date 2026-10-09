"""One explicit SDK point-cloud representation for mapping and live preview."""

CLOUD_SOURCES = ('raw', 'filtered')


def validate_cloud_source(value):
    if value not in CLOUD_SOURCES:
        raise ValueError('cloud_source 只支持 raw 或 filtered，不进行自动切换')
    return value


def selected_cloud_source(config):
    return validate_cloud_source(config.get('cloud_source', 'filtered'))


def source_frame_topic(side, source):
    if side not in ('left', 'right'):
        raise ValueError('点云源必须明确指定 left 或 right')
    return '/wc_mapping/lidar_'+side+('/source_frame' if validate_cloud_source(source) == 'raw' else '/source_frame_filtered')


def preview_cloud_topic(side, source):
    source_frame_topic(side, source)  # Same side/source validation as acquisition.
    return '/wc_mapping/lidar_'+side+('/points_raw' if source == 'raw' else '/points_filtered')


def cloud_description(source):
    return ('raw：主机滤波前的 SDK XYZ（已含深度解码和镜头投影），不是原始光学深度。'
            if validate_cloud_source(source) == 'raw' else 'filtered：当前完整主机滤波链后的 SDK XYZ。')
