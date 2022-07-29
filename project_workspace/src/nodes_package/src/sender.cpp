#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <iostream>

using namespace std;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_sender"); //Node creation
  ros::NodeHandle n; 
  ros::Publisher robot_pub = n.advertise<std_msgs::Float64>("/robot_arm/joint2_position_controller/command", 1000);

   ros::Rate loop_rate(1000);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  std_msgs::Float64 angle;
  while (ros::ok())
  {
 
    cout << "Write the degrees of the orientation" << endl;
    double angleValue;
    cin >> angleValue;
    angle.data = angleValue;
    robot_pub.publish(angle);
    cout << "published!" << endl;
  }

  return 0;
}
