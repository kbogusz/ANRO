cmake_minimum_required(VERSION 2.8.3)
project(lab1)

find_package(catkin REQUIRED COMPONENTS
	roscpp	
	rospy
	std_msgs
	message_generation
	genmsg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(lab1 src/lab1.cpp)
target_link_libraries(lab1 ${catkin_LIBRARIES})
add_dependencies(lab1 lab1_generate_messages_cpp)
