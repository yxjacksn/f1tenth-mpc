from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpc_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yxjacksn',
    maintainer_email='yxjacksn@seas.upenn.edu',
    description='MPC controller for F1TENTH',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node = mpc_pkg.node:main',
        ],
    },
)
