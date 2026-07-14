from setuptools import setup

package_name = "smartwheel_tests"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SmartWheel team",
    maintainer_email="todo@example.com",
    description="Cross-package mapping-v2 architecture and integration tests.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
