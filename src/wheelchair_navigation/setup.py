from setuptools import find_packages, setup
from glob import glob

package_name = "wheelchair_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="Named goal storage and goal publishing helpers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goal_manager_node = wheelchair_navigation.goal_manager_node:main",
            "navigation_status_node = wheelchair_navigation.navigation_status_node:main",
            "frontier_explorer_node = wheelchair_navigation.frontier_explorer_node:main",
        ],
    },
)
