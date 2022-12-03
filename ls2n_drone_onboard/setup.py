from setuptools import setup
import os
from glob import glob

package_name = 'ls2n_drone_onboard'

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
    maintainer='Damien SIX',
    maintainer_email='damien.six@ls2n.fr',
    description='Classic UAV embedded applications',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'observers = ls2n_drone_onboard.observers:main',
            'position_control = ls2n_drone_onboard.position_control:main',
            'velocity_joystick_control = ls2n_drone_onboard.velocity_joystick_control:main'
        ],
    },
)
