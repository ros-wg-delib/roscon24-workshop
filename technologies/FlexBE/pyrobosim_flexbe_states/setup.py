#!/usr/bin/env python

from glob import glob
from setuptools import setup
from setuptools import find_packages

PACKAGE_NAME = 'pyrobosim_flexbe_states'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + "/tests", glob('tests/*.test')),
        ('share/' + PACKAGE_NAME + "/launch", glob('tests/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@TODO.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_action_state = pyrobosim_flexbe_states.example_action_state',
            'example_state = pyrobosim_flexbe_states.example_state',
        ],
    },
)
