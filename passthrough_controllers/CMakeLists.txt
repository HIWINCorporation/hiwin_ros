cmake_minimum_required(VERSION 3.0.2)
project(pass_through_controllers)

find_package(catkin REQUIRED
COMPONENTS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  realtime_tools
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
# LIBRARIES
  CATKIN_DEPENDS
    control_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    trajectory_msgs
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pass_through_controllers.cpp
)
add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
