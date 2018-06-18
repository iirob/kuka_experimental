cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_hw_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  force_torque_sensor
  force_controllers
  kdl_parser
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kuka_hardware_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
    force_torque_sensor
    force_controllers
  DEPENDS
    TinyXML
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(kuka_hardware_interface
  src/kuka_hardware_interface.cpp
)
add_library(kuka_hardware_interface_with_fts
  src/kuka_hardware_interface_with_fts.cpp
)

target_link_libraries(kuka_hardware_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(kuka_hardware_interface_with_fts
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
  kuka_hardware_interface
)

add_executable(kuka_hardware_interface_node
  src/kuka_hardware_interface_node.cpp
)

target_link_libraries(kuka_hardware_interface_node
  kuka_hardware_interface
  kuka_hardware_interface_with_fts
)


install(
  TARGETS kuka_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_hardware_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
