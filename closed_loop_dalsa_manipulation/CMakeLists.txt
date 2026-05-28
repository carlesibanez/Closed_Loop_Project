cmake_minimum_required(VERSION 3.5)
project(closed_loop_dalsa_manipulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(pick_place_task src/pick_place_task.cpp)

# Link libraries
ament_target_dependencies(pick_place_task
  moveit_task_constructor_core
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  tf2_eigen
)

# Install executable
install(TARGETS
  pick_place_task
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(visual_servoing src/visual_servoing.cpp)
ament_target_dependencies(visual_servoing 
  rclcpp 
  geometry_msgs 
  std_srvs
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  visual_servoing
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
