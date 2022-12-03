from setuptools import setup
import os
from glob import glob

package_name = 'ls2n_mpc_single_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shiyu Liu',
    maintainer_email='shiyu.liu@ls2n.fr',
    description='Single Drone MPC Controller',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_mpc_node = ls2n_mpc_single_drone.mpc_controller:main'
        ],
    },
)
