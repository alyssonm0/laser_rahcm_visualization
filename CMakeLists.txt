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
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(usv_gazebo_thrust_plugin SHARED
  src/usv_gazebo_thrust_plugin.cc
)

ament_target_dependencies(usv_gazebo_thrust_plugin
  rclcpp
  gazebo_ros
  std_msgs
  sensor_msgs
  Boost
  Eigen3
)

target_link_libraries(usv_gazebo_thrust_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

if(build_testing)
  find_package(ament_lint_auto required)
  set(ament_cmake_copyright_found true)
  set(ament_cmake_cpplint_found true)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
