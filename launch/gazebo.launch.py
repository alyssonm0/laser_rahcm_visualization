import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_name = "laser_rahcm_visualization"
    pkg_share = get_package_share_directory(package_name)
    urdf_file_path = os.path.join(pkg_share, "urdf", "rahcm.urdf.xacro")
     
    # controller_yaml = os.path.join(pkg_share, "config", "rahcm_controllers.yaml") # Já estava comentado

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_file_path
    ])

    actions = []

    # Verifica e lança o Gazebo se necessário
    try:
        subprocess.check_call(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        gazebo_running = True
    except subprocess.CalledProcessError:
        gazebo_running = False

    # Se não estiver, inclui o launch do gazebo_ros (com plugin paths)
    if not gazebo_running:
        print("Gazebo server not running, launching it...")
        gazebo_launch_path = PathJoinSubstitution([
            get_package_share_directory('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ])
        
        world_path = PathJoinSubstitution([
            pkg_share,
            'worlds',
            'empty.world' 
        ])
 

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'world': world_path, 'verbose': 'false'}.items(),
        )
        actions.append(gazebo_launch)
    else:
        print("Gazebo server already running.")


    # robot_state_publisher
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    ))


    actions.append(TimerAction(
        period=5.0, 
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', package_name,
                    '-topic', 'robot_description', 
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '3.0', 
                    '-R', '0',
                    '-P', '0',
                    '-Y', '1.5708',
                ],
            )
        ]
    ))
    
    # actions.append(TimerAction(
    #     period=7.0,  
    #     actions=[
    #         Node(
    #             package='controller_manager',
    #             executable='spawner',
    #             arguments=[
    #                 'joint_state_broadcaster', 
    #                 '--controller-manager', '/controller_manager', 
    #                 '--controller-manager-timeout', '30' 
    #             ],
    #             output='screen',
    #         )
    #     ]
    # ))
    #
    # actions.append(TimerAction(
    #     period=9.0,  
    #     actions=[
    #         Node(
    #             package='controller_manager',
    #             executable='spawner',
    #             arguments=[
    #                 'arm_controller',
    #                 '--controller-manager', '/controller_manager',
    #                 '--controller-manager-timeout', '30'
    #             ],
    #             output='screen',
    #         )
    #     ]
    # ))

    return actions


def generate_launch_description():
    package_name = 'laser_rahcm_visualization' 
    pkg_share = get_package_share_directory(package_name)
    
    
    meshes_dir = os.path.join(pkg_share, 'meshes')
    gazebo_model_paths = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    new_gazebo_model_path = meshes_dir
    if gazebo_model_paths: 
        new_gazebo_model_path += os.pathsep + gazebo_model_paths

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=new_gazebo_model_path
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulation (Gazebo) clock if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
