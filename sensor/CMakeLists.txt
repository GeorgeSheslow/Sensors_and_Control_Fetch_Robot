cmake_minimum_required(VERSION 3.5)

project(sensor LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  #project_setup # Needed for the services
  roscpp
  sensor_msgs
  std_msgs
  #nav_msgs
  #rosbag # Needed for rosbag manipulation
  #roslib # Needed for ros::package::getPaths
)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_FLAG)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core REQUIRED)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}
)

###############
## Libraries ##
###############

## STUDENTS !!!
## Declare C++ libraries here
## Example provided with grid-processing (which you will most liekly use
## It is expected you would have more libraries for your work and ADD MORE lines
add_library(${PROJECT_NAME} src/main.cpp)

# FOR EXAMPLE
#add_library(${PROJECT_NAME}_my_library src/my_library.cpp)


###############
## Executable ##
###############

## Declare C++ executable
## STUDENTS !!!
## if you do have more cpp files that are not part of library add them here
## this is unlikely, so you would not need to change, otherwise add to end
add_executable(${PROJECT_NAME} src/main.cpp src/sensor.cpp)

## Add cmake target dependencies of the executable, enforces generating messages
## VERY VERY VERY HIGHLY unlikely to change
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## STUDENTS !!!
## It is expected you would have more libraries for your work and would need to REPLACE the below line
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME})


