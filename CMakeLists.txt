cmake_minimum_required(VERSION 3.0.2)
project(ur_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  hardware_interface
  joint_limits_interface
  kdl_parser
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  sensor_msgs
  speed_scaling_interface
  speed_scaling_state_controller
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
  ur_dashboard_msgs
  ur_msgs
  urdf
  visualization_msgs
  ur_robot_driver
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    ur_controllers
  CATKIN_DEPENDS
    actionlib
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    hardware_interface
    joint_limits_interface
    kdl_parser
    message_runtime
    pluginlib 
    realtime_tools
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    trajectory_msgs
    ur_robot_driver
    ur_dashboard_msgs
    ur_msgs
    urdf
    visualization_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  DEPENDS
    pinocchio
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


## Declare a C++ library
add_library(ur_controllers
  src/joint_admittance_controller.cpp
)

## Add cmake target dependencies of the library
add_dependencies(ur_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  # ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(ur_controllers PUBLIC
  pinocchio::pinocchio
  ${catkin_LIBRARIES}
)

target_include_directories(ur_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(ur_controllers PUBLIC
  include
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/ur_controllers_node.cpp)
# add_executable(ur_ft_calib_node
#   src/force_torque_calibration.cpp
#   src/force_torque_calibration_node.cpp
# )

# target_link_libraries(ur_ft_calib_node
#   ${catkin_LIBRARIES}
# )

# target_include_directories(ur_ft_calib_node PUBLIC
#   include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Installation
install(TARGETS ur_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(TARGETS ur_ft_calib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ur_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# catkin_install_python(
#   PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(ur_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(ur_controllers
    FILES ${SOURCES}
    DEPENDS ur_controllers
  )
endif()


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_controllers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
