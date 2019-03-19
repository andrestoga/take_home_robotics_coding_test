cmake_minimum_required(VERSION 2.8.3)
project(turtle_action)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
  actionlib
  actionlib_msgs
  geometry_msgs
  take_home_test
)

# add_action_files(
# 	DIRECTORY
# 	action
# 	FILES
# 	Move.action
# )

# generate_messages(
# 	DEPENDENCIES
# 	actionlib_msgs
# 	turtlesim
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_action
 # CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs take_home_test turtlesim roscpp
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

set(turtle_action_node_SRCS
  src/turtle_robot.cpp
  src/turtle_robot_action.cpp
)

set(turtle_action_node_HDRS
  include/turtle_action/turtle_robot.hpp
)

add_executable(turtle_action_node ${turtle_action_node_SRCS} ${turtle_action_node_HDRS})

## Specify libraries to link a library or executable target against
target_link_libraries(turtle_action_node
  ${catkin_LIBRARIES}
)

add_dependencies(turtle_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})