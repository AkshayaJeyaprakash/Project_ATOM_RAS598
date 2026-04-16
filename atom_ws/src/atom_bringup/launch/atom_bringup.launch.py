import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

os.environ['ROS_DISCOVERY_SERVER'] = '192.168.1.183:11811'

def generate_launch_description():

    robot_ns_arg = DeclareLaunchArgument('robot_namespace', default_value='robot_03', description='TurtleBot 4 namespace')
    robot_ns = LaunchConfiguration('robot_namespace')
    server_url_arg = DeclareLaunchArgument('server_url', default_value='http://192.168.1.154:5000', description='Inference server URL on host machine')
    server_url = LaunchConfiguration('server_url')

    return LaunchDescription([
        robot_ns_arg,
        server_url_arg,

        Node(package='atom_bringup', executable='camera_processor', name='camera_processor', parameters=[{'robot_namespace': robot_ns}], output='screen'),
        Node(package='atom_bringup', executable='lidar_processor', name='lidar_processor', parameters=[{'robot_namespace': robot_ns}], output='screen'),
        Node(package='atom_bringup', executable='object_detector', name='object_detector', parameters=[{'server_url': server_url}], output='screen'),
        Node(package='atom_bringup', executable='clip_scorer', name='clip_scorer', parameters=[{'server_url': server_url}], output='screen'),
        Node(package='atom_bringup', executable='semantic_map_builder', name='semantic_map_builder', parameters=[{'dedup_threshold': 0.5}], output='screen'),
        Node(package='atom_bringup', executable='exploration_coordinator', name='exploration_coordinator', output='screen'),
        Node(package='atom_bringup', executable='kinematics_node', name='kinematics_node', parameters=[{'wheel_radius':0.0352, 'wheelbase':0.233}], output='screen'),
        Node(package='atom_bringup', executable='goal_publisher', name='goal_publisher', parameters=[{'robot_namespace': robot_ns}], output='screen'),
        Node(package='atom_bringup', executable='vln_integration', name='vln_integration', parameters=[{'server_url': server_url}], output='screen'),
    ])