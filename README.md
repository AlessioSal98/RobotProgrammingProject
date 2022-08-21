# RobotProgrammingProject
The idea of this project is to use ROS and Gazebo in order to build and simulate a robot arm that is able to perform direct and inverse kinematics tasks


# How to run the project
## Ros and Gazebo
In order to run the project it is first necessary to boot Ros and Gazebo:
* Open a new terminal-->roscore
* Open a new terminal-->rosrun gazebo_ros gazebo
## Visualize the robot in gazebo
Once gazebo is open, it is necessary to spawn the robot inside it, this can be done in the following way:
* Open a new terminal
* Navigate in the project_workspace folder
* source devel/setup.bash
* roslaunch robot_package spawn.launch
The robot will now appear in Gazebo
## Start the Sender and Receiver nodes
