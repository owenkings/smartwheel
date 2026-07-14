from setuptools import find_packages, setup

package_name = "smartwheel_state_estimation"

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
    description="LIO-primary selector, wheel fallback, and residual diagnostics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_lio_node = smartwheel_state_estimation.mock_lio_node:main",
            "state_selector_node = smartwheel_state_estimation.state_selector_node:main",
        ]
    },
)

