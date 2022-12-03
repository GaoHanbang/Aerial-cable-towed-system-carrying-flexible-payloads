import shutil
import sys
from setuptools import setup
import os
from glob import glob
import subprocess

package_name = 'ls2n_drone_bridge'


def generate_model_list():
    data_files = []
    # List the models
    for path, dirs, files in os.walk("sitl_px4_simulation/models"):
        install_dir = os.path.join('share', package_name, path)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files


# Build PX4 for LS2N simulation and install specific LS2N simulation files in the appropriate folder
if 'install' in sys.argv:
    try:
        px4_folder = os.environ['PX4_SOURCE_DIR']
        if os.path.exists(px4_folder):
            # Disabling the lockstep (not working with ROS2 simulation for now)
            shutil.copy('sitl_px4_simulation/configuration/rtps.cmake',
                        os.path.join(px4_folder, 'boards/px4/sitl'))
            my_env = os.environ.copy()
            my_env["DONT_RUN"] = '1'
            p = subprocess.Popen(['make', 'px4_sitl_rtps', 'gazebo', '-j14'], cwd=px4_folder,
                                 env=my_env)
            p.wait()
            for file in glob('sitl_px4_simulation/configuration/9*'):
                shutil.copy(file,
                            os.path.join(px4_folder, 'build/px4_sitl_rtps/etc/init.d-posix/airframes'))
                shutil.copy('sitl_px4_simulation/configuration/rc.mc_simulation_ls2n',
                            os.path.join(px4_folder, 'build/px4_sitl_rtps/etc/init.d-posix'))
        else:
            raise FileNotFoundError
    except (KeyError, FileNotFoundError):
        print("Please define PX4_SOURCE_DIR with the path to your local PX4 folder.")

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
               ] + generate_model_list(),
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
