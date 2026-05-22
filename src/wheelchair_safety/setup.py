from setuptools import find_packages, setup

package_name = "wheelchair_safety"

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
    description="Safety supervision and velocity limiting.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_supervisor_node = wheelchair_safety.safety_supervisor_node:main",
            "velocity_limiter_node = wheelchair_safety.velocity_limiter_node:main",
            "emergency_stop_node = wheelchair_safety.emergency_stop_node:main",
        ],
    },
)
