# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug

# Include any dependencies generated for this target.
include CMakeFiles/sensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensor.dir/flags.make

CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o: CMakeFiles/sensor.dir/flags.make
CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o: sensor_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o -c /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/sensor_autogen/mocs_compilation.cpp

CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/sensor_autogen/mocs_compilation.cpp > CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.i

CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/sensor_autogen/mocs_compilation.cpp -o CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.s

CMakeFiles/sensor.dir/sensor.cpp.o: CMakeFiles/sensor.dir/flags.make
CMakeFiles/sensor.dir/sensor.cpp.o: /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sensor.dir/sensor.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensor.dir/sensor.cpp.o -c /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/sensor.cpp

CMakeFiles/sensor.dir/sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensor.dir/sensor.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/sensor.cpp > CMakeFiles/sensor.dir/sensor.cpp.i

CMakeFiles/sensor.dir/sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/sensor.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/sensor.cpp -o CMakeFiles/sensor.dir/sensor.cpp.s

CMakeFiles/sensor.dir/main.cpp.o: CMakeFiles/sensor.dir/flags.make
CMakeFiles/sensor.dir/main.cpp.o: /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/sensor.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensor.dir/main.cpp.o -c /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/main.cpp

CMakeFiles/sensor.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensor.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/main.cpp > CMakeFiles/sensor.dir/main.cpp.i

CMakeFiles/sensor.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor/main.cpp -o CMakeFiles/sensor.dir/main.cpp.s

# Object files for target sensor
sensor_OBJECTS = \
"CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/sensor.dir/sensor.cpp.o" \
"CMakeFiles/sensor.dir/main.cpp.o"

# External object files for target sensor
sensor_EXTERNAL_OBJECTS =

libsensor.so: CMakeFiles/sensor.dir/sensor_autogen/mocs_compilation.cpp.o
libsensor.so: CMakeFiles/sensor.dir/sensor.cpp.o
libsensor.so: CMakeFiles/sensor.dir/main.cpp.o
libsensor.so: CMakeFiles/sensor.dir/build.make
libsensor.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
libsensor.so: CMakeFiles/sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libsensor.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sensor.dir/build: libsensor.so

.PHONY : CMakeFiles/sensor.dir/build

CMakeFiles/sensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensor.dir/clean

CMakeFiles/sensor.dir/depend:
	cd /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor /home/paul/git/Sensors_and_Control_Fetch_Robot/sensor /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug /home/paul/git/Sensors_and_Control_Fetch_Robot/build-sensor-Desktop-Debug/CMakeFiles/sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensor.dir/depend

