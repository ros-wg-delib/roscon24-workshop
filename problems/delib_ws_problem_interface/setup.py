from setuptools import find_packages, setup

package_name = "delib_ws_problem_interface"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hec2le",
    maintainer_email="christian.henkel2@de.bosch.com",
    description="Problem interface for ROS 2 Deliberation Workshop",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
