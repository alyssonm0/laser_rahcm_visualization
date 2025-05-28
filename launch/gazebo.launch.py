import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_name = "laser_rahcm_visualization"
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, "urdf", "rahcm.urdf.xacro")

    # Gera descrição do robô
    robot_description = Command(["xacro ", urdf_file])

    actions = []

    # Verifica se o Gazebo está rodando
    gazebo_running = subprocess.call(["pgrep", "-x", "gzserver"], stdout=subprocess.DEVNULL) == 0

    # Se não estiver, inclui o launch do gazebo_ros (com plugin paths)
    if not gazebo_running:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                ])
            ),
            launch_arguments={
                "world": PathJoinSubstitution([
                    get_package_share_directory("gazebo_ros"),
                    "worlds",
                    "empty.world"
                ])
            }.items(),
        )
        actions.append(gazebo_launch)

    # Publica estado do robô
    actions.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    ))

    # Spawna entidade no Gazebo
    actions.append(Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", package_name,
            "-topic", "/robot_description",
            "-x", "-0.0", "-y", "0.0", "-z", "3.0",
            "-R", "0", "-P", "0", "-Y", "0",
        ],
    ))

    return actions


def generate_launch_description():
    package_name = "laser_rahcm_visualization"
    pkg_share = get_package_share_directory(package_name)
    meshes_dir = PathJoinSubstitution([pkg_share, 'meshes'])
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                os.pathsep,
                meshes_dir
            ]
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        OpaqueFunction(function=launch_setup)
    ])