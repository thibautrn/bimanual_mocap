from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bimanualrobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrelm',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_executor = bimanualrobot_bringup.joint_trajectory_executor:main',
            'joint_state_checker = bimanualrobot_bringup.joint_state_checker:main',
            'hardware_checker = bimanualrobot_bringup.hardware_checker:main',
        ],
    },
)