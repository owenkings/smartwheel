from setuptools import find_packages, setup

package_name = "wheelchair_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="Sensor adapter nodes and mock sensor publishers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xtm60_adapter_node = wheelchair_sensors.xtm60_adapter_node:main",
            "imu_adapter_node = wheelchair_sensors.imu_adapter_node:main",
            "ultrasonic_adapter_node = wheelchair_sensors.ultrasonic_adapter_node:main",
            "camera_adapter_node = wheelchair_sensors.camera_adapter_node:main",
            "mock_sensor_node = wheelchair_sensors.mock_sensor_node:main",
        ],
    },
)
