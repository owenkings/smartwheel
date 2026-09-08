from glob import glob

from setuptools import find_packages, setup

package_name = "smartwheel_global_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartWheel team",
    maintainer_email="todo@example.com",
    description="FAST-LIO2 input contract and mutually exclusive global mapping backends.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lio_input_adapter = smartwheel_global_mapping.lio_adapter_node:main",
            "cloud_to_scan_node = smartwheel_global_mapping.cloud_to_scan_node:main",
            "offline_replay_normalizer_node = smartwheel_global_mapping.offline_replay_normalizer_node:main",
            "offline_static_tf_relay_node = smartwheel_global_mapping.offline_static_tf_relay_node:main",
            "rtabmap_optimized_cloud_node = smartwheel_global_mapping.rtabmap_optimized_cloud_node:main",
        ]
    },
)
