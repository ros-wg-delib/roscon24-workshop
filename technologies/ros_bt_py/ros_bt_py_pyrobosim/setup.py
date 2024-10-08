import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ros_bt_py_pyrobosim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "trees"),
            glob(os.path.join("trees", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Oberacker",
    maintainer_email="oberacker@fzi.de",
    description="BT nodes for the ROSCon2024 workshop",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
