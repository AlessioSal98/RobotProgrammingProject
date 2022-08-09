#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/LinkStates.h>
#include <cmath>

using namespace std;

void end_effector_Callback(const gazebo_msgs::LinkStates& msg)
{
  float x = ceil(msg.pose[4].position.x*1000.0)/1000.0;
  float y = ceil(msg.pose[4].position.y*1000.0)/1000.0;
  float z = ceil(msg.pose[4].position.z*1000.0)/1000.0;
  cout << "END EFFECTOR POSITION:" << endl;
  cout <<  "X: " << x << endl;
  cout <<  "Y: " << y << endl;
  cout <<  "Z: " << z << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_listener"); //Node creation
  ros::NodeHandle n; 
  ros::Subscriber ee = n.subscribe("/gazebo/link_states", 1, end_effector_Callback);
  cout << "START:" << endl;
  ros::spin();

  return 0;
}
