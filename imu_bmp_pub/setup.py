from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'imu_bmp_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zephyr',
    maintainer_email='zephyr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_bmp_node = imu_bmp_pub.node:main',
            'pwm_and_valves = imu_bmp_pub.pwm_and_valves:main',
            'valve_teleop = imu_bmp_pub.valve_teleop:main',
        ],
    },
)
