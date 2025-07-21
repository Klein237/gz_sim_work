import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare parameter file path
    pkg_path = get_package_share_directory('ros_gz_bringup')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pkg_path, 'config', 'teleop.yaml'
        )
    )

    # joy_node: publishes sensor_msgs/msg/Joy on 'joy'
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[params_file]
    )

    # teleop_twist_joy: converts Joy -> Twist, publishes on 'cmd_vel'
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/diff_drive/cmd_vel')]
    )

   

    return LaunchDescription([
      
        joy_node,
        teleop_node,
    ])
