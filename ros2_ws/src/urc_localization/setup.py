import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'urc_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='urc@rover.local',
    description='GPS to odometry conversion for URC 2026 robot_localization fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_odom_node = urc_localization.gps_odom_node:main',
        ],
    },
)
