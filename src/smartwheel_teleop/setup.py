from setuptools import find_packages, setup

package_name = "smartwheel_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartWheel team",
    maintainer_email="todo@example.com",
    description="Headless WSAD controller with deadman, ramp, timeout, and hardware gate.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safe_teleop_node = smartwheel_teleop.node:main",
            "command_watchdog_node = smartwheel_teleop.watchdog_node:main",
        ]
    },
)

