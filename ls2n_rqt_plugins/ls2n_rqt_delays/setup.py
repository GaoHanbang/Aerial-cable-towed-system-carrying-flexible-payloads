from setuptools import setup

package_name = 'ls2n_rqt_delays'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
         ['resource/delays.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damien SIX',
    maintainer_email='damien.six@ls2n.fr',
    description='A plugin to check delays on drones',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delays = ' + package_name + '.main:main',
        ],
    },
)
