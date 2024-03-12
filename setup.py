import os
from glob import glob
from setuptools import setup

package_name = 'arm_control_marmotte'

setup(
    name='arm_control_marmotte',
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include all param files
        (os.path.join('share', package_name), glob('sequences/*'))
    ],
    install_requires=['setuptools', 'kortex_bringup', 'rclpy'],
    maintainer='William Dubois',
    maintainer_email='william.dubois@norlab.ulaval.ca',
    description='Marmotte Arm teleop package',
    license='BSD 3.0',
    entry_points={
        'console_scripts': [
            'arm_control_node = arm_control_marmotte.arm_control_node:main',
        ],
    },
)