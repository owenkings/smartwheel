"""Nav2 parameter fragments for the obstacle clouds (local costmap and collision monitor only).

Topics and frames (shadow replay; a live publisher must use the same):
``/obstacles/<side>/points`` (filtered obstacles, marking and clearing) and
``/obstacles/<side>/clearing`` (free-space ray ends, clearing only), both
sensor_msgs/PointCloud2 x y z float32 in frame ``lidar_<side>``, placed by a static
transform at each lidar's origin with base_link's axes. The cloud frame is the ray
origin for clearing, so each lidar is its own observation source. Without the
clearing topic, marks in front of open space (a long corridor) would only be
cleared by the footprint or the rolling window.

``obstacle_input`` selects what the local costmap and the collision monitor use:
``scan`` (default, unchanged), ``cloud`` (the clouds replace /scan there) or
``scan_and_cloud``. The global costmap is not touched here.
"""
from __future__ import annotations

from .obstacle_cloud import ObstacleFilter

OBSTACLE_INPUTS = ("scan", "cloud", "scan_and_cloud")
SIDES = ("left", "right")
CLOUD_TOPICS = {"left": "/obstacles/left/points", "right": "/obstacles/right/points"}
CLEAR_TOPICS = {"left": "/obstacles/left/clearing", "right": "/obstacles/right/clearing"}
CLOUD_FRAMES = {"left": "lidar_left", "right": "lidar_right"}
VOXEL_Z_RESOLUTION_M = 0.10
VOXEL_Z_CELLS = 16          # the Humble VoxelLayer maximum
CLEAR_RANGE_MARGIN_M = 0.5


def check(obstacle_input, sides):
    """Validated (obstacle_input, sides tuple)."""
    if obstacle_input not in OBSTACLE_INPUTS:
        raise ValueError("obstacle_input must be one of " + ", ".join(OBSTACLE_INPUTS))
    sides = tuple(sides or ())
    if obstacle_input == "scan":
        if sides:
            raise ValueError("obstacle_clouds needs obstacle_input cloud or scan_and_cloud")
        return obstacle_input, ()
    if not sides or len(set(sides)) != len(sides) or any(s not in SIDES for s in sides):
        raise ValueError("obstacle_clouds must name left and/or right once each")
    return obstacle_input, tuple(s for s in SIDES if s in sides)


def heights_in_base(cfg=ObstacleFilter()):
    """(min, max) obstacle height window in base_link z (the floor is axle_height_m below base_link)."""
    return cfg.floor_z_base + cfg.min_height_m, cfg.floor_z_base + cfg.max_height_m


def voxel_layer(sides, cfg=ObstacleFilter()):
    """VoxelLayer for the local costmap: per lidar a marking source and a clearing-only source.

    The voxel grid starts at the lowest obstacle height (layer 0 = 0.05-0.15 m above
    the floor). Clearing rays never end lower than ``clear_min_end_height_m``, so
    layer 0 is only cleared by the footprint or when it leaves the rolling window:
    low objects are remembered rather than erased by grazing floor rays.
    """
    low, high = heights_in_base(cfg)
    names = []
    layer = {"plugin": "nav2_costmap_2d::VoxelLayer", "enabled": True, "footprint_clearing_enabled": True,
             "max_obstacle_height": high, "origin_z": low, "z_resolution": VOXEL_Z_RESOLUTION_M,
             "z_voxels": VOXEL_Z_CELLS, "unknown_threshold": VOXEL_Z_CELLS, "mark_threshold": 0,
             "combination_method": 1, "publish_voxel_map": False}
    common = {"data_type": "PointCloud2", "sensor_frame": "", "obstacle_min_range": 0.0, "raytrace_min_range": 0.0,
              "expected_update_rate": 0.0, "observation_persistence": 0.0, "inf_is_valid": False,
              "clear_after_reading": False}
    for side in sides:
        layer["cloud_" + side] = dict(common, topic=CLOUD_TOPICS[side], marking=True, clearing=True,
                                      min_obstacle_height=low, max_obstacle_height=high,
                                      obstacle_max_range=cfg.max_range_m,
                                      raytrace_max_range=cfg.max_range_m + CLEAR_RANGE_MARGIN_M)
        layer["clear_" + side] = dict(common, topic=CLEAR_TOPICS[side], marking=False, clearing=True,
                                      min_obstacle_height=cfg.floor_z_base - 1.0, max_obstacle_height=3.0,
                                      obstacle_max_range=cfg.max_range_m,
                                      raytrace_max_range=cfg.clear_max_range_m + CLEAR_RANGE_MARGIN_M)
        names += ["cloud_" + side, "clear_" + side]
    layer["observation_sources"] = " ".join(names)
    return layer


def local_plugins(obstacle_input):
    if obstacle_input == "cloud":
        return ["voxel_layer", "inflation_layer"]
    return ["obstacle_layer", "voxel_layer", "inflation_layer"]


def monitor_sources(sides, cfg=ObstacleFilter()):
    """Collision-monitor pointcloud sources, one per lidar (heights in base_link)."""
    low, high = heights_in_base(cfg)
    return {"cloud_" + s: {"type": "pointcloud", "topic": CLOUD_TOPICS[s], "min_height": low, "max_height": high,
                           "enabled": True} for s in sides}


def monitor_source_names(obstacle_input, sides):
    clouds = ["cloud_" + s for s in sides]
    return clouds if obstacle_input == "cloud" else ["scan"] + clouds
