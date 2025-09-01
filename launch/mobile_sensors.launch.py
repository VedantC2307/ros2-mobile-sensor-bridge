import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_directory = get_package_share_directory('mobile_sensor')

    # Path to your main JS file (now in server folder)
    start_js_file = os.path.join(
        share_directory,
        'dist',
        'server',
        'index.js')
    
    cert_directory = os.path.join(share_directory, 'dist')

    sensor_node = Node(
        name='mobile_sensor_node',
        executable='node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            start_js_file
        ],
        cwd=cert_directory)

    ld = LaunchDescription()
    ld.add_action(sensor_node)

    return ld