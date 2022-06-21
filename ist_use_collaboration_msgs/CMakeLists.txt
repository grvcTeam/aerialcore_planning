cmake_minimum_required(VERSION 3.1.2)
project(ist_use_collaboration_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geographic_msgs
  sensor_msgs
  message_generation
  roscpp
  genmsg
  actionlib_msgs
)

# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

add_action_files(
   FILES
   RequestMobileChargingStation.action
   DoCloserInspection.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geographic_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gauss_msgs
  CATKIN_DEPENDS message_runtime std_msgs geographic_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
