cmake_minimum_required(VERSION 3.7.2)
project(iksolver)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
# set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
# message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

###############
## Packages ##
###############
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    gazebo_ros
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

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
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
  CATKIN_DEPENDS
    control_toolbox
    gazebo_plugins
    gazebo_ros
    robot_controllers
    robot_controllers_interface
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  LIBRARIES
    fetch_gazebo_plugin
  DEPENDS
    EIGEN3
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
target_link_libraries(${PROJECT_NAME}_ex ${catkin_LIBRARIES})