"""Launch wrapper for the external LIVO backend (FAST-LIVO2 / R3LIVE).

This package does NOT contain the LIVO algorithm. This wrapper reads
config/livo_interface.yaml, and:
  * backend:=none  -> starts nothing external (sensors/fusion/EKF still run).
  * backend:=fast_livo2|r3live -> includes that backend's launch file with the
    remaps from livo_interface.yaml, IF the package is installed.
  * if the selected backend package or launch file is missing, it prints a
    clear error and exits without crashing the rest of the system.
"""
import os

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _setup(context, *args, **kwargs):
    iface_path = LaunchConfiguration("livo_interface").perform(context)
    backend = LaunchConfiguration("backend").perform(context).strip()
    try:
        with open(iface_path) as f:
            iface = yaml.safe_load(f) or {}
    except OSError as exc:
        return [LogInfo(msg=f"[livo_3d_mapping] ERROR: cannot read {iface_path}: {exc}")]

    if not backend:
        backend = str(iface.get("backend", "none"))
    if backend == "none":
        return [LogInfo(msg="[livo_3d_mapping] backend:=none -> no external LIVO backend started. "
                            "Point cloud fusion, sensors, EKF and projection still run.")]

    backends = iface.get("backends", {})
    if backend not in backends:
        return [LogInfo(msg=f"[livo_3d_mapping] ERROR: unknown backend '{backend}'. "
                            f"Known: {sorted(backends)} or 'none'.")]

    bcfg = backends[backend] or {}
    ext_pkg = LaunchConfiguration("external_package").perform(context).strip() or bcfg.get("package", "")
    ext_launch = LaunchConfiguration("external_launch_file").perform(context).strip() or bcfg.get("launch_file", "")
    if not ext_pkg or not ext_launch:
        return [LogInfo(msg=f"[livo_3d_mapping] ERROR: backend '{backend}' has no package/launch_file. "
                            f"Set them in {iface_path} or pass external_package:= external_launch_file:=.")]

    try:
        ext_share = get_package_share_directory(ext_pkg)
    except (PackageNotFoundError, ValueError):
        return [LogInfo(msg=f"[livo_3d_mapping] ERROR: backend '{backend}' selected but ROS package "
                            f"'{ext_pkg}' is NOT installed. Install FAST-LIVO2/R3LIVE (see "
                            f"docs/fast_livo2_r3live_integration.md) or relaunch with backend:=none.")]

    launch_path = os.path.join(ext_share, "launch", ext_launch)
    if not os.path.isfile(launch_path):
        return [LogInfo(msg=f"[livo_3d_mapping] ERROR: package '{ext_pkg}' found but launch file "
                            f"'{launch_path}' does not exist. Fix external_launch_file in {iface_path}.")]

    remaps = iface.get("external_remappings", []) or []
    set_remaps = [SetRemap(src=str(p[0]), dst=str(p[1]))
                  for p in remaps if isinstance(p, (list, tuple)) and len(p) == 2]
    return [
        LogInfo(msg=f"[livo_3d_mapping] starting backend '{backend}' from {launch_path} "
                    f"with {len(set_remaps)} remap(s)."),
        GroupAction(set_remaps + [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
        ]),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("wheelchair_3d_mapping")
    default_iface = os.path.join(pkg_share, "config", "livo_interface.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="",
                              description="fast_livo2 | r3live | none. Empty = use value in livo_interface.yaml."),
        DeclareLaunchArgument("livo_interface", default_value=default_iface),
        DeclareLaunchArgument("external_package", default_value="",
                              description="Override the backend ROS package name."),
        DeclareLaunchArgument("external_launch_file", default_value="",
                              description="Override the backend launch file name."),
        OpaqueFunction(function=_setup),
    ])
