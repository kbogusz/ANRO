cmake_minimum_required(VERSION 2.8.3)
project(lab5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  genmsg 
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   oint_point.srv
   oint_trajectory.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ocmd1 src/ocmd.cpp)
add_executable(oint1 src/oint.cpp)
add_executable(ikin src/ikin.cpp)
target_link_libraries(ocmd1 ${catkin_LIBRARIES})
target_link_libraries(oint1 ${catkin_LIBRARIES})
target_link_libraries(ikin ${catkin_LIBRARIES})
add_dependencies(ocmd1 lab5_generate_messages_cpp)
add_dependencies(oint1 lab5_generate_messages_cpp)
add_dependencies(ikin lab5_generate_messages_cpp)
