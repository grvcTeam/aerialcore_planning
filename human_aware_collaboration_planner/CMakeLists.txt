# 1. Required CMake Version
cmake_minimum_required(VERSION 3.5)

# 2. Package Name
project(human_aware_collaboration_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  genmsg
  geometry_msgs
  actionlib_msgs
  #mrs_msgs
  #mrs_lib
  actionlib
  behaviortree_cpp_v3
  uav_abstraction_layer
  mrs_actionlib_interface
  #roscpp
  mavros_msgs
  #YAML_CPP
)

# 4. Enable Python module suport (catkin_python_setup())

# 5. Message/Service/Action Generators (add_message/service/action_files())
add_message_files(
  FILES
  AgentBeacon.msg
  PlannerBeacon.msg
  Waypoint.msg
  MonitorParams.msg
  InspectParams.msg
  DeliverToolParams.msg
  RechargeParams.msg
  WaitParams.msg
  Task.msg
  HumanTarget.msg
  Position.msg
  Tool.msg
  BatteryControl.msg
  MissionOver.msg
)

#add_service_files(
#  FILES
#)

add_action_files(
  DIRECTORY action 
  FILES 
  NewTask.action 
  NewTaskList.action
  BatteryEnough.action
  TaskResult.action
)

# 6. Invoke message/service/action generation (generate_messages())
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  #mrs_msgs
  mrs_actionlib_interface
)

# 7. Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME} #MrsActionlibInterface
  CATKIN_DEPENDS behaviortree_cpp_v3 message_runtime mrs_actionlib_interface 
  #CATKIN_DEPENDS behaviortree_cpp_v3 message_runtime mrs_actionlib_interface roscpp mrs_lib mrs_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 8. Libraries/Executables to build (add_library(), add_executable()/target_link_libraries())

## Declare a C++ library
add_library(${PROJECT_NAME} src/classes.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(gesture_recognition_faker src/gesture_recognition_faker.cpp)
target_link_libraries(gesture_recognition_faker ${catkin_LIBRARIES})
add_dependencies(gesture_recognition_faker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(high_level_planner src/high_level_planner.cpp)
target_link_libraries(high_level_planner yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(high_level_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(high_level_planner PROPERTIES COMPILE_FLAGS "-g")

add_executable(agent_behaviour_manager src/agent_behaviour_manager.cpp)
target_link_libraries(agent_behaviour_manager yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(agent_behaviour_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(agent_behaviour_manager PROPERTIES COMPILE_FLAGS "-g")

#add_executable(lower_level_controllers src/lower_level_controllers.cpp)
#target_link_libraries(lower_level_controllers yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(lower_level_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(battery_faker src/battery_faker.cpp)
target_link_libraries(battery_faker yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(battery_faker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 9. Tests to build (catkin_add_gtest())

# 10. Install rules (install())


