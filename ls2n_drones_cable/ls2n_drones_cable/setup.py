from setuptools import setup
import os
from glob import glob

package_name = 'ls2n_drones_cable'


def generate_model_list():
    data_files = []
    # List the models
    for path, dirs, files in os.walk("models"):
        install_dir = os.path.join('share', package_name, path)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
                   (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + generate_model_list(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hanbang GAO',
    maintainer_email='gaohanbang@gmail.com',
    description='Simulation/Control law for 2 drones with one cable',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drones_cable = ls2n_drones_cable.drones_cable:main',
        ],
    },
)
