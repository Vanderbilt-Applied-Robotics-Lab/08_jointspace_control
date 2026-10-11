cmake_minimum_required(VERSION 3.8)
project(jointspace_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

## INCLUDE(S)
include_directories(include)

# executables
add_executable(jointspace_control src/jointspace_control.cpp)

# dependencies
ament_target_dependencies(jointspace_control rclcpp moveit_ros_planning_interface tf2 tf2_ros)

install(TARGETS jointspace_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
