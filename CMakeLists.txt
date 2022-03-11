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
  cv_bridge
  geometry_msgs
  behaviortree_cpp_v3
  cameras_cpp
  message_filters
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
)

add_library(${PROJECT_NAME}
  src/visual_behavior/ifball.cpp
  src/visual_behavior/PIDController.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  include
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})



add_library(asr_ifball_bt_node SHARED src/visual_behavior/ifball.cpp)
list(APPEND plugin_libs
  asr_ifball_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(followball_node 
  src/followball_node
)
target_link_libraries(followball_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${ZMQ_LIBRARIES})


roslint_cpp(
 src/followball_node.cpp
 include/visual_behavior/ifball.h src/visual_behavior/Ifball.cpp
)