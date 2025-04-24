import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    package_name = "laser_rahcm_visualization"
    urdf_file_name = "rahcm.urdf.xacro"
    package_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_path, "urdf", urdf_file_name)

    robot_description = Command(["xacro ", urdf_path])

    actions = []

    # Verifica se o Gazebo está rodando
    gazebo_running = subprocess.call(["pgrep", "-x", "gzserver"], stdout=subprocess.DEVNULL) == 0

    # Se não estiver rodando, abre o empty.world
    if not gazebo_running:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "world": os.path.join(
                    get_package_share_directory("gazebo_ros"), "worlds", "empty.world"
                )
            }.items(),
        )
        actions.append(gazebo_launch)

    # Publica a descrição do robô
    actions.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    ))

    # Spawna o robô na posição e orientação desejada
    actions.append(Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "laser_rahcm_visualization",
            "-topic", "/robot_description",
            "-x", "-19.0",
            "-y", "-5.0",
            "-z", "0.5",
            "-R", "0",
            "-P", "0",
            "-Y", "1.5708",
        ]
    ))


    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        OpaqueFunction(function=launch_setup)
    ])
