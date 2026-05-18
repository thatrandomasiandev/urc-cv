import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'urc_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='urc@rover.local',
    description='Mission state machine for URC 2026 autonomous waypoint and ArUco navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = urc_autonomy.mission_node:main',
            'waypoint_loader = urc_autonomy.waypoint_loader:main',
            'estop_node = urc_autonomy.estop_node:main',
            'telemetry_node = urc_autonomy.telemetry_node:main',
        ],
    },
)
