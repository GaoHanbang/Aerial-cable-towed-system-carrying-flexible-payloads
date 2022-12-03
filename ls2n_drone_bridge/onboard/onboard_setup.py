import shutil
import sys
from setuptools import setup
import os
from glob import glob
import subprocess

package_name = 'ls2n_drone_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
                   ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
                   ('share/' + package_name, ['package.xml']),
                   (os.path.join('share', package_name), glob('launch/*.launch.py')),
                   (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
               ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damien Six',
    maintainer_email='damien.six@ls2n.fr',
    description='ROS2 bridges with LS2N drones',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_bridge = ls2n_drone_bridge.drone_bridge:main'
        ],
    },
)
