#!/usr/bin/env python

from glob import glob

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'pyrobosim_flexbe_utilities'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Pyrobosim utilities used by pyrobosim_flexbe_states ',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
