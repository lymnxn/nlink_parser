from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
        package='nlink_parser',
        executable='linktrack_aoa',
        output="screen",
        parameters=[{'port_name':'/dev/ttyUSB0', 
        'baud_rate':921600}]
    )])