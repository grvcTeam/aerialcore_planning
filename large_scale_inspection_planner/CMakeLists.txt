cmake_minimum_required(VERSION 2.8.3)
project(large_scale_inspection_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  roslib
  mission_lib
  path_planner
  aerialcore_msgs
)

find_package(PythonLibs 2.7)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES 
  CATKIN_DEPENDS roscpp rospy tf roslib mission_lib path_planner aerialcore_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_controller src/mission_controller_node.cpp src/mission_controller.cpp src/centralized_planner.cpp src/parameter_estimator.cpp src/plan_monitor.cpp)
target_include_directories(mission_controller PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mission_controller ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(mission_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
