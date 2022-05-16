cmake_minimum_required(VERSION 3.7.2)
project(fetch_robot_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    fetch_gazebo
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf2_geometry_msgs
    message_generation
)

catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${CATKIN_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_message_files(
  FILES
  Location_3D.msg
  RGB_Image_Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_install_python(PROGRAMS scripts/fetch_sensor.py
                              scripts/fetch_gui.py

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_package(
  LIBRARIES
    fetch_gazebo_plugin
  INCLUDE_DIRS
  CATKIN_DEPENDS 
    message_runtime
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui
  PATTERN ".svn" EXCLUDE
)
