from setuptools import find_packages, setup


setup(
    name="smartwheel_workbench",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/smartwheel_workbench"]),
        ("share/smartwheel_workbench", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SmartWheel team",
    maintainer_email="todo@example.com",
    description="Mock-only RViz workbench backend.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["workbench_node = smartwheel_workbench.node:main"]},
)
