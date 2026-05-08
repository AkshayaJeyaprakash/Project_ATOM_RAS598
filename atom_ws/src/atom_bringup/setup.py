from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'atom_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'atom_bringup', 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eva',
    maintainer_email='nivaspiduru@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_processor = atom_bringup.camera_processor:main',
            'goal_publisher = atom_bringup.goal_publisher:main',
            'object_detector = atom_bringup.object_detector:main',
            'exploration_coordinator = atom_bringup.exploration_coordinator:main',
            'memory_mapper = atom_bringup.memory_mapper:main',
            'safety_monitor = atom_bringup.safety_monitor:main',
        ],
    },
)