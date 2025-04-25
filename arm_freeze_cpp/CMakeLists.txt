cmake_minimum_required(VERSION 3.0.2)
project(arm_freeze_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  multirobot_services
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ArmFreeze.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES arm_freeze_cpp
 CATKIN_DEPENDS roscpp actionlib actionlib_msgs message_runtime moveit_core moveit_ros_planning moveit_ros_planning_interface tf2 tf2_ros multirobot_services
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/arm_freeze_server.cpp
  src/arm_freeze_client.cpp
)

# Server executable
add_executable(arm_freeze_server_node src/arm_freeze_server_node.cpp)
target_link_libraries(arm_freeze_server_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Client executable
add_executable(arm_freeze_client_node src/arm_freeze_client_node.cpp)
target_link_libraries(arm_freeze_client_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_freeze_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_freeze_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install executables and/or libraries
install(TARGETS ${PROJECT_NAME} arm_freeze_server_node arm_freeze_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)