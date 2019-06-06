cmake_minimum_required(VERSION 2.8.3)
project(multi_agent_plan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  message_generation
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  map_server
  roslib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  Pose.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  GetPlan.srv
  Set2DPose.srv
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
  std_msgs  # Or other packages containing msgs
  std_srvs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_agent_plan
 CATKIN_DEPENDS roscpp message_runtime message_generation geometry_msgs std_msgs std_srvs visualization_msgs tf2 tf2_ros roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/multi_agent_plan.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(multi_agent_node_SRCS
  src/agent.cpp
  src/agent_node.cpp
  src/agent_node_app.cpp
)

set(multi_agent_node_HDRS
  include/multi_agent_plan/agent_node.h
)

set(planner_node_SRCS
  src/manhattan_planner.cpp
  src/planner_node_app.cpp
)

set(planner_node_HDRS
  include/multi_agent_plan/cell.h
  include/multi_agent_plan/cell_dfs.h
  include/multi_agent_plan/cell_dijk.h
  include/multi_agent_plan/map.h
  include/multi_agent_plan/planner_2d.h
  include/multi_agent_plan/det_planner_2d.h
  include/multi_agent_plan/manhattan_planner.h
  include/multi_agent_plan/dfs.h
  include/multi_agent_plan/dijkstra.h
  include/multi_agent_plan/planner_node.h
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(multi_agent_node ${multi_agent_node_SRCS} ${multi_agent_node_HDRS})
add_executable(planner_node ${planner_node_SRCS} ${planner_node_HDRS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(multi_agent_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(planner_node ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(multi_agent_node
  ${catkin_LIBRARIES}
)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multi_agent_plan.cpp)

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} )

#   add_dependencies(
#     ${PROJECT_NAME}-test
#     ${PROJECT_NAME}
#     ${catkin_EXPORTED_TARGETS}
#   )

# endif()

if(CATKIN_ENABLE_TESTING)
  message("\n\nGoing to build the rostest!\n\n")
  find_package(rostest REQUIRED)
  add_rostest_gtest(ros_testing_agent_planner test/ros_testing_agent_planner.launch test/ros_testing_agent_planner.cpp)
  target_link_libraries(ros_testing_agent_planner ${catkin_LIBRARIES})
  add_dependencies(
    ros_testing_agent_planner
    planner_node
    multi_agent_node
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )

  catkin_add_gtest(${PROJECT_NAME}-test test/test_multi_agent_plan.cpp src/manhattan_planner.cpp )
  target_link_libraries(${PROJECT_NAME}-test map_server_image_loader ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)