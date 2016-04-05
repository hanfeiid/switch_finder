cmake_minimum_required(VERSION 2.8.3)
project(switch_finder)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Point.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS opencv2
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(switch_finder src/switch_finder.cpp src/detectId.cpp src/imgSubAndTagPub.cpp src/ARTag.cpp)
target_link_libraries(switch_finder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_dependencies(switch_finder switch_finder_generate_messages_cpp)
