cmake_minimum_required(VERSION 3.8)
project(laser_rahcm_visualization)

# Encontra todos os pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

# Instalação do arquivo URDF
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Instalação dos arquivos de lançamento
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Garante que todas as dependências sejam encontradas e configuradas corretamente
ament_package()
