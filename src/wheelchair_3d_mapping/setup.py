from glob import glob

from setuptools import find_packages, setup

package_name = "wheelchair_3d_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="LiDAR-Visual-Inertial-Wheel 3D SLAM integration nodes and launch files.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dual_lidar_cloud_fusion_node = wheelchair_3d_mapping.dual_lidar_cloud_fusion_node:main",
            "wheel_livo_consistency_monitor = wheelchair_3d_mapping.wheel_livo_consistency_monitor:main",
            "cloud_to_occupancy_grid_node = wheelchair_3d_mapping.cloud_to_occupancy_grid_node:main",
            "rgb_cloud_colorizer_node = wheelchair_3d_mapping.rgb_cloud_colorizer_node:main",
        ],
    },
)
