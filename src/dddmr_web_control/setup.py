from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "dddmr_web_control"
static_files = [path for path in glob("static/*") if Path(path).is_file()]
vendor_files = [path for path in glob("static/vendor/*") if Path(path).is_file()]
config_files = [path for path in glob("config/*.yaml") if Path(path).is_file()]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", config_files),
        (f"share/{package_name}/static", static_files),
        (f"share/{package_name}/static/vendor", vendor_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Offline web console for localization and point-to-point navigation.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_control_node = dddmr_web_control.web_control_node:main",
        ],
    },
)
