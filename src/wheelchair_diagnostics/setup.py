from setuptools import find_packages, setup

package_name = "wheelchair_diagnostics"

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
    description="Hardware self-check, runtime watchdog and localization health nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_self_check_node = wheelchair_diagnostics.hardware_self_check_node:main",
            "sensor_watchdog_node = wheelchair_diagnostics.sensor_watchdog_node:main",
            "localization_health_node = wheelchair_diagnostics.localization_health_node:main",
        ],
    },
)
