from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "lab1_robot_description"
    rviz_file_name = "odom.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )
    single_track = Node(
            package=package_name,
            executable='single_track.py',
            parameters=[
                {'pose':[9.0, 0.0, 1.57079632679]}
            ]
    )
    double_track = Node(
            package=package_name,
            executable='double_track.py',
            parameters=[
                {'pose':[9.0, 0.0, 1.57079632679]}
            ]
    )
    yaw_rate = Node(
            package=package_name,
            executable='yaw_rate.py',
            parameters=[
                {'pose':[9.0, 0.0, 1.57079632679]}
            ]
    )
    gps = Node(
            package=package_name,
            executable='gps_emulator.py',
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", rviz_file_path
        ],
        output = "screen"
    )
    ekf_double = Node(package=package_name,
                      namespace='/double_track',
                      executable='ekf_node.py',
                      name='double_track_ekf',
                      parameters=[
                            {'pose':[9.0, 0.0, 1.57079632679]},
                            {'noise':[0.0494, 0.0870]}
                        ]
                      )

    ekf_single = Node(package=package_name,
                      namespace='/single_track',
                      executable='ekf_node.py',
                      name='single_track_ekf',
                      parameters=[
                            {'pose':[9.0, 0.0, 1.57079632679]},
                            {'noise':[0.0494, 0.0401]}
                        ]
                      )

    ekf_yawrate = Node(package=package_name,
                      namespace='/yaw_rate',
                      executable='ekf_node.py',
                      name='yaw_rate_ekf',
                      parameters=[
                            {'pose':[9.0, 0.0, 1.57079632679]},
                            {'noise':[0.0479, 0.0428]}
                        ]
                      )

    return LaunchDescription([
        single_track,
        double_track,
        yaw_rate,
        # gps,
        # ekf_double,
        # ekf_single,
        # ekf_yawrate,
        rviz
    ])