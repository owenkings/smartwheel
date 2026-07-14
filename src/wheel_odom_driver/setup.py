from setuptools import find_packages, setup

package_name = "wheel_odom_driver"

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
    description="Parameterized encoder-count differential odometry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["wheel_odom_node = wheel_odom_driver.node:main"]},
)

