# Sensors_and_Control_Fetch_Robot

High Level:

Have Fetch Robot grasp simple shapes

Tasks Broken Down into:

- GUI and simulation code
- object detection
- IK solver
- Inteegrstion and testing

Packages:

- fetch_gazebo (download via github)
- fetch_robot_sim (GUI, sim and obj_detect) - python
- iksolver - cpp

Software:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

# How to run:
*Ensure you have run "catkin build" and "source devel/setup.bash" beforehand
1. roslaunch fetch_robot_sim fetch.launch
2. roslaunch iksolver iksolver_node.launch
3. Start the simulation by pressing the "start" button in the gazebo window

# Simulation Environment

The system is run in the gazebo simulator.
The fetch.launch file launches gazebo empty world, fetch robot, table and grasping objects (Red Cylinder, Green Cube and Blue Cube)
Grasping objects and table was created by ourselves.
The fetch robot was taken from the fetch_gazebo package
On top of this there are launch files for each node and whole package for testing.

# GUI

The GUI is able to teleop the robot, send arm commands, display detected object information and run the inverse kinematics solver.
The GUI was created in python using PyQt5 was is a better option than the MATLAB GUI option.
The GUI was a key component when intergrating and debugging the system.

The GUI consists of the main gui.py file which controls the user inputs.
On top of this there are additional files stated below:
- fetch.py which takes control of the robot arm joint positions and ros messages to control the fetch robot
- teleop.py handles the user teleop input via sliders and buttons
- cameras.py handles the bounding image display on the GUI, this is run on a thread to reduce latency in the GUI

# Object Detection
Object Detection is run with the GUI and is updated every time the robot is not moving.
RGB-D camera from the Fetch robot is used for object detection and visual servoing. 
RGB camera is used to detect the object and visual servoing whilst returning the center point of the object. It also labels the objects.
OpenCV is used for the locations in the camera and the colours. The locations origin (0,0) is on the top left of the camera and x being the horizonal axis and y being the vertical axis.

The main files fetch_sensor.py and it is used for RGB-D sensor and calculations and reurns x,y,z for the inverse kinematics solver to solve.

# Inverse Kinematics Solver

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
