cmake_minimum_required(VERSION 3.8)
project(laser_rahcm_visualization)

# Encontra todos os pacotes necessários\ nfind_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(laser_rahcm_simulation REQUIRED)

# Executável do Action Client para mover braços
add_executable(arm_controller src/arm_controllers.cpp)
ament_target_dependencies(arm_controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)


# Instala os executáveis
install(TARGETS
  arm_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Instalação do arquivo URDF e outras pastas
install(DIRECTORY
  launch
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
