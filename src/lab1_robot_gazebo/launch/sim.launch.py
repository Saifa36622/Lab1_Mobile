#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import launch_ros.actions

def generate_launch_description():
    package_name = "lab1_robot_description"
    model_path =  get_package_share_directory(package_name) + '/models'
    rviz_file_name = "gazebo.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    world_file_name = "basic.world"
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        world_file_name
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "lab1_robot.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time":"true"}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                )
            ]
        ),
        launch_arguments={
            "world": world_file_path
        }.items()
    )
    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "example",
            "-x", "9.0",
            "-y", "0.0",
            "-z", "0.0",
            "-Y", "1.57079632679"],
        output = "screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )
    model_odom = Node(
            package=package_name,
            executable='model_odom.py',
    )

    delayed_model_odom = TimerAction(
        period=3.0,  # Delay in seconds (adjust if needed)
        actions=[model_odom]
    )

    # Static Transform Publisher (world -> odom)
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen"
    )
    # Launch!
    return LaunchDescription(
        [   
            SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
            gazebo,
            spawn_entity,
            rsp,
            rviz,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            position_controller_spawner,
            delayed_model_odom,
            static_tf
        ]
    )