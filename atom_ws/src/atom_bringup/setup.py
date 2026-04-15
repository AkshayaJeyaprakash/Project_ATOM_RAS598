from setuptools import find_packages, setup

package_name = 'atom_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/atom_bringup.launch.py']),
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
            # Perception nodes
            'lidar_processor = atom_bringup.lidar_processor:main',
            'camera_processor = atom_bringup.camera_processor:main',

            # AI nodes
            'object_detector = atom_bringup.object_detector:main',
            'clip_scorer = atom_bringup.clip_scorer:main',

            # Core custom nodes
            'semantic_map_builder = atom_bringup.semantic_map_builder:main',
            'exploration_coordinator = atom_bringup.exploration_coordinator:main',
            'kinematics_node = atom_bringup.kinematics_node:main',

            # Navigation
            'goal_publisher = atom_bringup.goal_publisher:main',

            # VLN bridge (refactored)
            'vln_integration = atom_bringup.vln_integration:main',
        ],
    },
)