from setuptools import find_packages, setup

package_name = "wheelchair_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="Perception nodes for point cloud projection and obstacle status.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pointcloud_to_laserscan_node = wheelchair_perception.pointcloud_to_laserscan_node:main",
            "obstacle_detector_node = wheelchair_perception.obstacle_detector_node:main",
            "dynamic_obstacle_layer_node = wheelchair_perception.dynamic_obstacle_layer_node:main",
            "passability_analyzer_node = wheelchair_perception.passability_analyzer_node:main",
        ],
    },
)
