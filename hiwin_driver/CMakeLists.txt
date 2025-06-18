cmake_minimum_required(VERSION 3.0.2)
project(hiwin_driver)

message(STATUS "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED 
  COMPONENTS
    control_msgs
    controller_manager
    hardware_interface
    industrial_robot_status_interface
    pluginlib
    pass_through_controllers
    roscpp
    std_srvs
)

find_package(hiwin_robot_client_library)
if (hiwin_robot_client_library_VERSION VERSION_LESS "0.0.4")
  message(FATAL_ERROR "hiwin_robot_client_library version >= 0.0.4 is required. Found ${hiwin_robot_client_library_VERSION}")
endif()

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    hiwin_driver_plugin
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    pluginlib
    pass_through_controllers
    roscpp
    std_srvs
  DEPENDS
    hiwin_robot_client_library
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hiwin_driver_plugin
  src/hardware_interface.cpp
)
target_link_libraries(hiwin_driver_plugin ${catkin_LIBRARIES} hiwin_robot_client_library::hrsdk)
add_dependencies(hiwin_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(hiwin_driver_node
  src/hardware_interface.cpp
  src/hardware_interface_node.cpp
)
target_link_libraries(hiwin_driver_node ${catkin_LIBRARIES} hiwin_robot_client_library::hrsdk)
add_dependencies(hiwin_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_states_without_extra_node
  src/joint_states_without_extra_node.cpp
)
target_link_libraries(joint_states_without_extra_node ${catkin_LIBRARIES})

install(TARGETS hiwin_driver_plugin hiwin_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)