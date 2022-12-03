from setuptools import setup
import os
from glob import glob
from setuptools.command.install import install
from ls2n_tools.trajectory_tools import generate_trajectories


class GenerateTrajectories(install):
    def run(self):
        install.run(self)
        generate_trajectories()


package_name = 'ls2n_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'trajectories'), glob('trajectories/*.traj')),
        (os.path.join('share', package_name), glob('launch/*'))
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='Damien SIx',
    maintainer_email='damien.six@ls2n.fr',
    description='Several useful tools for LS2N drones',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_trajectory = ls2n_tools.trajectory_tools:plot_trajectory',
            'list_trajectories = ls2n_tools.trajectory_tools:list_trajectories',
            'generate_trajectories = ls2n_tools.trajectory_tools:generate_trajectories',
            'moving_average = ls2n_tools.moving_average:main'
        ],
    },
    cmdclass={'install': GenerateTrajectories}
)
