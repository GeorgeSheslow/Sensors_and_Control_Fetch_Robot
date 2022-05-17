# Sensors_and_Control_Fetch_Robot

Software:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

Packages:

- download fetch_gazebo and add to catkin_ws

- Ensure all dependicies are downloaded (including PyQt5, python3)

# fetch_robot_sim

1. Try roslaunch fetch_gazebo simulation.launch

# iksolver

iksolver is a rosservice used to calculate a trajectory from the robot's current position to a requested position.
This service runs with name ``/calc_traj``. This can be seen with ``rosservice info /calc_traj``
It can be tested with ``rosservice call /calc_traj [0.66,0.00033,0.27]``

MoveIT:
- In catkin_ws/src/moveit/moveit_ros/planning/planning_components_tools/CMakeLists.txt, comment out lines 17 to 22
- Comment the line 75 in file catkin_ws/src/moveit_tutorials/CMakeLists.txt
- rviz_visual_tools must be installed.
- Boost must be installed
- Fix dependancies with ``rosdep install --from-paths src --ignore-src -r -y``
- Set up catkin build parameters with ``catkin config --cmake-args -DCATKIN_ENABLE_TESTING=0``
- Must be compiled with: ``catkin build``

Notes:
- All packages MUST contain a "include" folder, whether or not it would be empty anyway.
- catkin build MUST be used rather than catkin_make

1. Launch the robot state handler: ``roslaunch iksolver iksolver_ex.launch``
2. Launch the robot planner for inverse kinematics: ``roslaunch iksolver iksolver_node.launch``

# sensor


In ~/catkin_ws do catkin build
run source devel/setup.bash
1. run ``cd src/Sensor_and_Control_Fetch_Robot/fetch_robot_sim/scripts``
2. run ``python3 gui.py`` and press ``Grasp Prep``
3. repeat steps 1-3 in another terminal
4. run the code ``python3 sensor.py``
5. watch magic unfold
Notes: Use depth.py is integrated with sensor and is not needed x and y is accurate? z is not accurate at all as it depends the distacne from the object and the camera's
