cmake_minimum_required(VERSION 2.8.3)
project(large_scale_inspection_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# add_definitions(-DUSING_MS_TSP_PLANNER) # Comment this (and the line below) if not using the ms_tsp_planner.
# add_definitions(-DUSING_ORTOOLS)        # Comment this (and the lines below) if not using the OR-Tools.

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  roslib
  mission_lib
  path_planner
  aerialcore_msgs
  # ms_tsp_planner  # Comment this if not using ms_tsp_planner.
)

find_package(PythonLibs 2.7)

# set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)

# ATTENTION: REPLACE THE ORTOOLS_ROOT PATH BELOW FOR THE PATH WHERE YOU PLACED THE ORTOOLS BINARIES. In this case ~/ortools
# Comment these following lines if not using OR-Tools.
# set(ORTOOLS_ROOT "$ENV{HOME}/ortools")
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${ORTOOLS_ROOT}")
# find_package(ortools REQUIRED)

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
# Comment these following lines if not using OR-Tools.
# include_directories(
#   ${ORTOOLS_INCLUDE_DIRS}
# )

add_executable(mission_controller src/mission_controller_node.cpp src/mission_controller.cpp src/centralized_planner.cpp src/parameter_estimator.cpp src/plan_monitor.cpp)
target_include_directories(mission_controller PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mission_controller ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${CURL_LIBRARIES})
# target_link_libraries(mission_controller ${ORTOOLS_LIBRARIES})            # Comment this if not using the OR-Tools.
add_dependencies(mission_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
