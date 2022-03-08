cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

#find_package(ZMQ)
#if(ZMQ_FOUND)
#    message(STATUS "ZeroMQ found.")
#    add_definitions(-DZMQ_FOUND)
#else()
#  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
#endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  kobuki_msgs
  sensor_msgs
  roslint
  geometry_msgs
  behaviortree_cpp_v3
  cameras_cpp
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs behaviortree_cpp_v3 cameras_cpp geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/visual_behavior/ifball.cpp
)

add_executable(followball_node src/followball_node)
target_link_libraries(followball_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${ZMQ_LIBRARIES})

roslint_cpp(
 src/followball_node.cpp
 src/visual_behavior/Ifball.cpp
 include/visual_behavior/ifball.h
)