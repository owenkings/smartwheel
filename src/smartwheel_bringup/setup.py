from glob import glob

from setuptools import setup

package_name = "smartwheel_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/config",
            ["../../config/hardware_profile.mock.yaml", "../../config/hardware_profile.template.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartWheel team",
    maintainer_email="todo@example.com",
    description="Mapping-v2 launch composition and profiles.",
    license="Apache-2.0",
)
