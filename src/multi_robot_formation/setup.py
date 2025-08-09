from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='Multi-robot formation control using AprilTags for leader-follower behavior',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector = multi_robot_formation.apriltag_detector:main',
            'follower_localization = multi_robot_formation.follower_localization:main',
            'formation_controller = multi_robot_formation.formation_controller:main',
        ],
    },
)
