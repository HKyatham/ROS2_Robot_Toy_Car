# ROS2_Robot_Toy_Car
The aim of this project is understand how to do the following

Design a robot car using Solidworks.
Convert it to a URDF (Universal Robot Description Format).
Apply inertial physics to it, add a LIDAR sensor, controllers for motion control
Create a ROS package containing the XACRO files ready for launch
Write the appropriate launch files and run the robot in a simulated Gazebo world and to also control it through teleop

# Dependencies
This project is made using ROS 2

1. Firstly make sure you have ROS installed in your system. If not, visit http://wiki.ros.org/ROS/Installation and install ROS. If you already have ROS installed in your system, make sure to setup the sources list and setup your keys. Visit http://wiki.ros.org/noetic/Installation/Ubuntu (Incase you're using ROS Galactic)

2. Update your linux environment

    ```sudo apt update```

    ```sudo apt upgrade```

3. Install the necessary controllers packages using the following command. (change the version of ROS as per your installation)

    ```sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control```

    ```sudo apt-get install ros-galactic-controller-manager```

# Usage
### Launching the robot in a Gazebo world and teleoperating it.
1. Create a folder. create src folder in this directory.

   ```mkdir -p ros_ws/src```

2. Navigate to the newly created folder.

   ```cd ros_ws```

3. Copy the contents of this repository in the src folder.
4. Build the code using the ROS2 build command.

   ```colcon build```
5. Source the newly built files so that ROS can detect the new packages.

   ```source install/setup.bash```
6. Launch the Robot_model using the launch file, in case you want to launch the robot in Gazebo only.

    ```ROS2 launch Robot_model gazebo.launch.py```

     Launching both Gazebo and Rviz.
   
     ```ROS2 launch robot_model debug.launch.py```

7. Launch the Teleop controller. Open a new terminal and type the below command.

    ```ROS2 run robot_model_control robot_teleop```

     Now use thhe key board keys to provide the necessary veloccity and steering angle.
     ```
      -w forward velocity
      -s backward velocity
      -d left steering angle
      -a right steering angle
      -q Stop the robot

8. Control the robot using a PID controller. Open a new terminal and type the below command.

    ```ROS2 run robot_model_control robot_p_control```
