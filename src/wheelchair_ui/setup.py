from setuptools import find_packages, setup
from glob import glob

package_name = "wheelchair_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/static", glob("wheelchair_ui/static/*")),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn", "PyYAML"],
    zip_safe=True,
    maintainer="wheelchair team",
    maintainer_email="todo@example.com",
    description="FastAPI web UI for the wheelchair stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wheelchair_ui = wheelchair_ui.app:main",
        ],
    },
)
