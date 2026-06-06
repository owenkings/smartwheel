from glob import glob

from setuptools import find_packages, setup

package_name = "wheelchair_voice_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="Voice command parser and model API stubs.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_parser_node = wheelchair_voice_agent.command_parser_node:main",
            "audio_io_bridge_node = wheelchair_voice_agent.audio_io_bridge_node:main",
        ],
    },
)
