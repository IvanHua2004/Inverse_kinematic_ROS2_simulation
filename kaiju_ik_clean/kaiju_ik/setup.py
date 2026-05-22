from setuptools import setup
from glob import glob
import os

package_name = 'kaiju_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='IK demo with turtlesim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kaiju_ik_node = kaiju_ik.kaiju_ik_node:main',
            'target_mover = kaiju_ik.target_spawner:main',
        ],
    },
)
