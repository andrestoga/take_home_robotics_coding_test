cmake_minimum_required(VERSION 2.8.3)
project(take_home_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	turtlesim
	actionlib
	message_generation
)

add_action_files(
	FILES
	Move.action
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	turtlesim
)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	actionlib
	turtlesim
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(puppet_master src/puppet_master.cpp)

target_link_libraries(puppet_master
	${catkin_LIBRARIES}
)

add_dependencies(puppet_master ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS puppet_master
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
