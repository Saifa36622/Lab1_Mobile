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
    rviz_EKF_file_name = "EKF.rviz"
    rviz_EKF_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_EKF_file_name
    )
    start_pose = [9.0, 0.0, 1.57079632679] #x,y,yaw

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
            "-x", str(start_pose[0]),
            "-y", str(start_pose[1]),
            "-z", "0.0",
            "-Y", str(start_pose[2])],
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
    controller = Node(
            package=package_name,
            executable='controller2.py',
    )

    # Static Transform Publisher (world -> odom)
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen"
    )

    #Odom and EKF
    single_track = Node(
            package=package_name,
            executable='single_track.py',
            parameters=[
                {'pose':start_pose}
            ]
    )
    double_track = Node(
            package=package_name,
            executable='double_track.py',
            parameters=[
                {'pose':start_pose}
            ]
    )
    yaw_rate = Node(
            package=package_name,
            executable='yaw_rate.py',
            parameters=[
                {'pose':start_pose}
            ]
    )
    gps = Node(
            package=package_name,
            executable='gps_emulator.py',
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_EKF_file_path
        ],
        output = "screen"
    )
    ekf_double = Node(package=package_name,
                      namespace='/double_track',
                      executable='ekf_node.py',
                      name='double_track_ekf',
                      parameters=[
                            {'pose':start_pose},
                            {'noise':[0.0494, 0.0870]}
                        ]
                      )

    ekf_single = Node(package=package_name,
                      namespace='/single_track',
                      executable='ekf_node.py',
                      name='single_track_ekf',
                      parameters=[
                            {'pose':start_pose},
                            {'noise':[0.0494, 0.0401]}
                        ]
                      )

    ekf_yawrate = Node(package=package_name,
                      namespace='/yaw_rate',
                      executable='ekf_node.py',
                      name='yaw_rate_ekf',
                      parameters=[
                            {'pose':start_pose},
                            {'noise':[0.0479, 0.0428]}
                        ]
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
            controller,
            static_tf,
            #Odom and EKF
            single_track,
            double_track,
            yaw_rate,
            gps,
            ekf_double,
            ekf_single,
            ekf_yawrate,
            rviz2
        ]
    )