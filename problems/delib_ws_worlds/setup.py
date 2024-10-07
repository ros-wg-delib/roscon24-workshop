from glob import glob

from setuptools import find_packages, setup

package_name = "delib_ws_worlds"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Christian Henkel",
    maintainer_email="christian.henkel2@de.bosch.com",
    description="Example worlds for the ROS 2 Deliberation Workshop",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run = delib_ws_worlds.run:main",
            "is_at_goal = delib_ws_worlds.is_at_goal:main",
        ],
    },
)
