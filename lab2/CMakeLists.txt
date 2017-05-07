cmake_minimum_required(VERSION 2.8.3)
project(lab2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(NONKDL_DKIN src/NONKDL_DKIN.cpp)
target_link_libraries(NONKDL_DKIN ${catkin_LIBRARIES})
add_dependencies(NONKDL_DKIN lab2_generate_messages_cpp)

add_executable(KDL_DKIN src/NONKDL_DKIN.cpp)
target_link_libraries(KDL_DKIN ${catkin_LIBRARIES})
add_dependencies(KDL_DKIN lab2_generate_messages_cpp)


