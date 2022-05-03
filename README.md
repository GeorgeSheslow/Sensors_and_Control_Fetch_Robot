# Sensors_and_Control_Fetch_Robot

Software:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

Packages:

- download fetch_gazebo and add to catkin_ws

- Ensure all dependicies are downloaded

MoveIT:
- In catkin_ws/src/moveit/moveit_ros/planning/planning_components_tools/CMakeLists.txt, comment out lines 17 to 22
- Comment the line 75 in file catkin_ws/src/moveit_tutorials/CMakeLists.txt
- Must be compiled with: catkin build -DCATKIN_ENABLE_TESTING=0
- rviz_visual_tools must be installed.
- Boost must be installed
- Fix dependancies with rosdep install --from-paths src --ignore-src -r -y

Notes:
- All packages MUST contain a "include" folder, whether or not it would be empty anyway.
- catkin build MUST be used rather than catkin_make

1. Launch: roslaunch fetch_moveit_config demo.launch
2. Launch: roslaunch iksolver iksolver_ex.launch
