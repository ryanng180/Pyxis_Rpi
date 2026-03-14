from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensors_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pyxispi5',
    maintainer_email='pyxispi5@todo.todo',
    description='Sensors bringup package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'imu_node         = sensors_bringup.imu_node:main',
            'tilt_tf_node     = sensors_bringup.tilt_tf_node:main',
            'scan_filter_node = sensors_bringup.scan_filter_node:main',
            'proximity_node   = sensors_bringup.proximity_node:main',
        ],
    },
)
