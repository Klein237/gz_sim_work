# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    use_joy = LaunchConfiguration('use_joy')
    declare_use_joy = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Enable joystick teleop'
    )

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'tugbot', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'tugbot_warehouse.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    #joystick teleop
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(use_joy)
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'tugbot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    static_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base_link',
        output='screen',
        arguments=[
            '0', '0', '0',    
            '0', '0', '0',    
            'map',            
            'odom'       
        ],
    )

    odm_to_tf = Node(
        package='ros_gz_application',
        executable='odom_to_tf',
        name='odom_to_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],

    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
        declare_use_joy,
        joystick,
        odm_to_tf,
                            
        bridge,
        ros_gz_image_bridge,
        robot_state_publisher,
        static_map_to_base,
        rviz
    ])
