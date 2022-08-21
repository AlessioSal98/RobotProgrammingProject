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

# Description of the nodes
## Receiver node
The receiver node is the simplest of the two developed nodes as it works as a read only interface that constantly prints the position of the robot end effector in terms of 3D coordinates.
The main function instantiates a ROS subsriber object that subscribes to the */gazebo/link_states* topic.
A callback function is then used for printing the coordinates.
## Sender node
The sender node is the main program of this project as it works as an interface between the user, that will provide some commands for the robot, and the ROS topics that will handle the position of the individual joints of the robot.
In order to mantain some information strictly related to the robot that is being used (such as the direct kinematics or the jacobian of the defined 3R elbow-type robot) a new class Robot3R has been defined.
The sender node will provide a menu with the following commands that is possible to give to the robot:
* Direct Kinematics
* Inverse Kinematics (Analytical)
* Inverse Kinematics (Numerical)

### Direct Kinematics
This is the simplest functionality of the node, because it allows to specify the orientation of the 3 joints directly from keyboard and then send the information to the relative ROS topic, which are, in this case:
* */robot_arm/joint1_position_controller/command*
* */robot_arm/joint2_position_controller/command*
* */robot_arm/joint3_position_controller/command*

The information relative to each individual node, is stored inside a *std_msgs::Float64* object
### Inverse Kinematics (Analytical)
This part of the node asks the user to specify, from keyboard, the desired coordinates that the end effector of the robot should reach.
First, a check about the feasibilty of the task is made, by ensuring that the desired coordinates are inside the workspace of the robot.
If the task is feasible, then the *Robot3R::analyticalInverseKinematics* is called, and it will produce 4 possible inverse kinematics solutions, that will correspond to 4 joint configurations. In particular, the provided solution for this robot type are:
* Forward/Elbow Up
* Forward/Elbow Down
* Backward/Elbow up
* Backward/Elbow down

Once those solutions are computed, it is possible to choose one among them and then the joint configuration will be properly published on the relative topics
### Inverse Kinematics (Numerical)
This is another type of Inverse Kinematics that allows to find one of the possible inverse kinematics solution by applying a numerical method.
In this case, the used method is the Gradient Method that, given a start configuration, a value alpha, an error tollerance epsilon and the robot jacobian(which is defined in the Robot3R class) will provide as output a possible joint configuration that solves the required inverse kinematics task

