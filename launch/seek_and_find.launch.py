#!/usr/bin/env python3
"""Launch Gazebo with the seek-and-find arena, spawn TurtleBot3, and start
the YOLO-World vision-guided mission controller.

Run with:
    export TURTLEBOT3_MODEL=waffle_pi
    source /opt/ros/jazzy/setup.bash
    ros2 launch <path-to-this-file>/seek_and_find.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE_SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(PACKAGE_SRC_DIR, 'worlds', 'seek_and_find_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))

    # The AI node depends on ultralytics/torch, which live in a dedicated
    # virtualenv (yolo_venv, created with --system-site-packages so it can
    # still see rclpy / cv_bridge from the ROS install). We launch it as a
    # plain process with that venv's interpreter instead of a `ros2 run`
    # entry point, and delay it a few seconds so Gazebo + the robot spawn
    # (and the first camera frames) are ready first.
    venv_python = os.path.expanduser('~/colcon_ws/yolo_venv/bin/python3')
    node_script = os.path.join(PACKAGE_SRC_DIR, 'pai_final_task', 'seek_and_find_node.py')

    seek_and_find_cmd = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[venv_python, node_script],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(seek_and_find_cmd)

    return ld
