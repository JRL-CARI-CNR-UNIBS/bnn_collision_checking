cmake_minimum_required(VERSION 3.0.2)
project(bnn_collision_checking)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  eigen_conversions
  eigen_matrix_utils
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  name_sorting
  roscpp
  moveit_planning_helper
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs eigen_conversions eigen_matrix_utils moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools name_sorting roscpp moveit_planning_helper
  DEPENDS
)
include_directories(
   ${catkin_INCLUDE_DIRS}
)

add_executable(        compute_distance src/compute_distance.cpp)
add_dependencies(      compute_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( compute_distance
 ${catkin_LIBRARIES}
)
