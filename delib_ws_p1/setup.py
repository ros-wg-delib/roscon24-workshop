from glob import glob

from setuptools import find_packages, setup

package_name = 'delib_ws_p1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/data", glob('data/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Henkel',
    maintainer_email='christian.henkel2@de.bosch.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = delib_ws_p1.run:main'
        ],
    },
)
