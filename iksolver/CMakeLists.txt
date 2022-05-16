cmake_minimum_required(VERSION 3.7.2)
project(iksolver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
# set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
# message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

###############
## Packages ##
###############
find_package(catkin REQUIRED
  COMPONENTS
    fetch_robot_sim
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    message_generation
)

add_service_files(
  FILES
  calcTraj.srv
)

generate_messages(DEPENDENCIES std_msgs fetch_robot_sim moveit_msgs)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED) # EXACT 1.79.0 COMPONENTS system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    message_runtime
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

###############
## Executable ##
###############

## Declare C++ executable
add_executable(${PROJECT_NAME}_ex src/main.cpp)

## Add cmake target dependencies of the executable, enforces generating messages
## VERY VERY VERY HIGHLY unlikely to change
add_dependencies(${PROJECT_NAME}_ex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## STUDENTS !!!
## It is expected you would have more libraries for your work and would need to REPLACE the below line
target_link_libraries(${PROJECT_NAME}_ex ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY doc/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
PATTERN ".svn" EXCLUDE)