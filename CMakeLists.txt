cmake_minimum_required(VERSION 2.8.3)
project(visual_behavior)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  kobuki_msgs
  sensor_msgs
  roslint
  cv_bridge
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  cv_bridge
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  include
)

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
target_link_libraries(followball_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/followball_node.cpp
 include/visual_behavior/ifball.h src/visual_behavior/Ifball.cpp
)