import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "laser_rahcm_visualization"
    urdf_file_name = "rahcm.urdf.xacro"

    package_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_path, "urdf", urdf_file_name)

    robot_description = Command([
        "xacro ", urdf_path
    ])

    spawn_entity = Node(
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
)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        spawn_entity,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
