# RobotProgrammingProject
The idea of this project is to use ROS and Gazebo in order to build and simulate a robot arm that is able to perform direct and inverse kinematics tasks


# How to run the project
## Build the project
As first step, it is necessary to build the project, this can be done as follows:
* Open a new terminal
* Navigate in the project_workspace folder
* Command: catkin build
## ROS and Gazebo
In order to run the project it is also necessary to boot ROS and Gazebo:
* Open a new terminal
* Command: roscore
* Open a new terminal
* Command: rosrun gazebo_ros gazebo
## Spawn the robot in gazebo
Once gazebo is open, it is necessary to spawn the robot inside it, this can be done in the following way:
* Open a new terminal
* Navigate in the project_workspace folder
* Command: source devel/setup.bash
* Command: roslaunch robot_package spawn.launch
The robot will now appear in Gazebo
## Start the Sender and Receiver nodes
The sender and receiver nodes are the main component of this project, since they are used for the communication in ROS.
In order to run them is is simply necessary to execute the following steps:
* Open a new terminal (one for each of the two nodes)
* Navigate in the project_workspace folder
* Command: source devel/setup.bash
* Command: rosrun nodes_package sender (rosrun nodes_package receiver for the receiver node)

# How to use the nodes
## Receiver node
The receiver node is the simplest of the two developed nodes as it works as a read only interface that constantly prints the position of the robot end effector in terms of 3D coordinates.
The main function instantiates a ROS subsriber object that subscribes to the */gazebo/link_states* topic.
A callback function is then used for printing the coordinates.
## Sender node
The sender node is the main program of this project as it works as an interface between the user, that will provide some commands for the robot, and the ROS topics that will handle the position of the individual joints of the robot
