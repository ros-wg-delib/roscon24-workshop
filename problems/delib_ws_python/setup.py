from setuptools import find_packages, setup

import os
from glob import glob

package_name = "delib_ws_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hec2le",
    maintainer_email="christian.henkel2@de.bosch.com",
    description="Simple Python solutions to ROS 2 Deliberation Workshop",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["run = delib_ws_python.run:main"],
    },
)
