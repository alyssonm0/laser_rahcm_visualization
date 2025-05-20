cmake_minimum_required(VERSION 3.8)
project(laser_rahcm_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(laser_rahcm_simulation REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

add_library(usv_gazebo_thrust_plugin 
  src/usv_gazebo_thrust_plugin.cc
)

target_link_libraries(usv_gazebo_thrust_plugin
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
target_compile_options(usv_gazebo_thrust_plugin PRIVATE "-Wno-unknown-pragmas")

install(TARGETS usv_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  urdf
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(usv_gazebo_thrust_plugin)
ament_export_dependencies(gazebo rclcpp std_msgs Eigen3 Boost sensor_msgs)

ament_package()
