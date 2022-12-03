from setuptools import setup
import os
from glob import glob

package_name = 'ls2n_drone_command_center'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('ls2n_drone_command_center/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damien Six',
    maintainer_email='damien.six@ls2n.fr',
    description='A package containing the tools required to remotely control one or several drones at the LS2N '
                'laboratory.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = ls2n_drone_command_center.joystick_connect:main',
            'trajectory_publisher = ls2n_drone_command_center.trajectory_publisher:main',
            'qualisys = ls2n_drone_command_center.qualisys_bridge:main',
            'multi_control = ls2n_drone_command_center.multi_control:main'
        ],
    },
)
