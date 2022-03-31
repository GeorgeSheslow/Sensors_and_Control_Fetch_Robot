cmake_minimum_required(VERSION 3.5)

project(sensor LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core REQUIRED)

add_library(sensor SHARED
  sensor_global.h
  sensor.cpp
  sensor.h
)

target_link_libraries(sensor PRIVATE Qt5::Core)

target_compile_definitions(sensor PRIVATE SENSOR_LIBRARY)
