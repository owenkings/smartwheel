from setuptools import find_packages, setup

package_name = "wheelchair_voice_agent"

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
    description="Voice command parser and model API stubs.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_parser_node = wheelchair_voice_agent.command_parser_node:main",
        ],
    },
)
