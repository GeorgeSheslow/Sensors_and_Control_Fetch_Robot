Sensors_and_Control_Fetch_Robot
Software:

Ubuntu 20.04
ROS Noetic
Gazebo 11

Packages:
download fetch_gazebo and add to catkin_ws
Ensure all dependicies are downloaded

How to run:
1. In ~/catkin_ws do catkin build
2. run source devel/setup.bash
3. do "cd src/Sensor_and_Control_Fetch_Robot/fetch_robot_sim/scripts"
4. run the code "python3 gui.py" and press prep arm
5. repeat steps 1-3 in another terminal
6. run the code "python3 sensor.py"
7. watch magic unfold

Notes:
Use depth.py is integrated with sensor and is not needed
x and y is accurate?
z is not accurate at all as it depends the distacne from the object and the camera's
