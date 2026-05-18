from setuptools import find_packages, setup

package_name = 'search_behavior'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='urc@rover.local',
    description='Structured ArUco search pattern at GPS waypoints for URC 2026',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_node = search_behavior.search_node:main',
        ],
    },
)
