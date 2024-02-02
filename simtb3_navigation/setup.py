from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simtb3_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = simtb3_navigation.interactive_waypoint_follower:main',
            'gps_waypoints_planner = simtb3_navigation.gps_waypoints_planner:main',
            'gps_waypoints_follower = simtb3_navigation.gps_waypoints_follower:main',
            'map2d_waypoints_planner = simtb3_navigation.map2d_waypoints_planner:main',
            'map2d_waypoints_follower = simtb3_navigation.map2d_waypoints_follower:main',
        ],
    },
)
