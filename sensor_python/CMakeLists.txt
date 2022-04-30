cmake_minimum_required(VERSION 3.7.2)
project(sensor_python)
find_package(catkin REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)


find_package(catkin
  REQUIRED
    gazebo_ros
    fetch_gazebo
    roscpp
    rospy
    std_msgs
    message_generation

)

add_message_files(
  FILES
  Location.msg
  Location_3D.msg
  Object_Info.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


# catkin_package(
#   CATKIN_DEPENDS
#     control_toolbox
#     gazebo_plugins
#     gazebo_ros
#     robot_controllers
#     robot_controllers_interface
#   LIBRARIES
#     fetch_gazebo_plugin
#   INCLUDE_DIRS
#     include
# )

# add_library(fetch_gazebo_plugin src/plugin.cpp)
# target_link_libraries(fetch_gazebo_plugin
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# install(
#   TARGETS fetch_gazebo_plugin
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# install(
#   PROGRAMS
#     scripts/prepare_simulated_robot.py
#     scripts/prepare_simulated_robot_pick_place.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_install_python(PROGRAMS
#   scripts/test.py
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

# install(
#   DIRECTORY config include launch robots worlds models
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(
#   DIRECTORY config include launch robots worlds models
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)
