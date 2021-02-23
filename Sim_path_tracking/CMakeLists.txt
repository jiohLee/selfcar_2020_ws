cmake_minimum_required(VERSION 2.8.3)
project(Sim_path_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  morai_msgs
  tf
  ackermann_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Sim_path_tracking
  CATKIN_DEPENDS roscpp  std_msgs nav_msgs tf ackermann_msgs morai_msgs
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(Sim_path_tracking 
 src/main.cpp
 ../SC_Control/msg/selfcar_cmd.msg)
target_link_libraries(Sim_path_tracking  ${catkin_LIBRARIES} )

add_executable(ochang_Sim_path_tracking 
 src/main.cpp
 ../SC_Control/msg/selfcar_cmd.msg)
target_link_libraries(ochang_Sim_path_tracking  ${catkin_LIBRARIES} )


add_executable(Sim_path_tracking_only_lane
 src/main_lane.cpp
 ../SC_Control/msg/selfcar_cmd.msg)
target_link_libraries(Sim_path_tracking_only_lane  ${catkin_LIBRARIES} )








