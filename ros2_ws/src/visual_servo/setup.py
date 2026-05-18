from setuptools import find_packages, setup

package_name = 'visual_servo'

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
    description='Visual servo controller for URC 2026 autonomous navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = visual_servo.servo_node:main',
        ],
    },
)
