from setuptools import setup
import os
from glob import glob

package_name = 'wall_following_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aung Kaung Myat',
    maintainer_email='103877301+AungKaung1928@users.noreply.github.com',
    description='Reactive wall following for TurtleBot3 with PD control and collision avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = wall_following_project.wall_follower_controller:main',
        ],
    },
)