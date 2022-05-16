cmake_minimum_required(VERSION 3.7.2)
project(fetch_messages)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
  FILES
  Location_3D.msg
  RGB_Image_Info.msg
)

add_service_files(
  FILES
  IkInfo.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# install(
#   DIRECTORY msg srv
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

catkin_package(
  CATKIN_DEPENDS message_runtime
)