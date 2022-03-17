cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  kobuki_msgs
  sensor_msgs
  roslint
  cv_bridge
  geometry_msgs
  behaviortree_cpp_v3
  cameras_cpp
  message_filters
  darknet_ros_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  cv_bridge
  behaviortree_cpp_v3
  cameras_cpp
  geometry_msgs
  message_filters
  darknet_ros_msgs
)
find_package(BehaviorTreeV3)
find_package(OpenCV)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(visual_behavior src/visual_behavior/PIDController.cpp)

add_library(asr_ifball_bt_node SHARED src/visual_behavior/ifball.cpp)
target_link_libraries(asr_ifball_bt_node visual_behavior)

add_library(asr_ifperson_bt_node SHARED src/visual_behavior/ifperson.cpp)
target_link_libraries(asr_ifperson_bt_node visual_behavior)

add_library(asr_move_bt_node SHARED src/visual_behavior/Move.cpp)

list(APPEND plugin_libs
  asr_ifball_bt_node
  asr_ifperson_bt_node
  asr_move_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(followball_node 
  src/followball_node.cpp
)
add_executable(followperson_node 
  src/followperson_node.cpp
)
add_executable(followboth_node 
  src/followboth_node.cpp
)
target_link_libraries(followball_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(followperson_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(followboth_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

roslint_cpp(
 src/followball_node.cpp
 src/followperson_node.cpp
 src/followboth_node.cpp
 include/visual_behavior/PIDController.hpp src/visual_behavior/PIDController.cpp
 include/visual_behavior/ifball.h src/visual_behavior/ifball.cpp
 include/visual_behavior/ifperson.h src/visual_behavior/ifperson.cpp
 include/visual_behavior/Move.h src/visual_behavior/Move.cpp
)