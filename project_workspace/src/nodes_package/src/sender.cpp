#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include "vec_f.h"

using namespace std;

bool checkPointInsideWorkspace(double* coords){
  double origin[] = {0,0,2};
  double radius = 2;
  double diffX = coords[0]-origin[0];
  double diffY = coords[1]-origin[1];
  double diffZ = coords[2]-origin[2];
  bool res;
  if(((diffX*diffX)+(diffY*diffY)+(diffZ*diffZ))<=(radius*radius))
  {
    res = true;
  }else{
    res = false;
  }
  return res;
}

double * analyticalInverseKinematics(double* coords){
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Cobot_Sender"); //Node creation
  ros::NodeHandle n; 
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher joint1 = n.advertise<std_msgs::Float64>("/robot_arm/joint1_position_controller/command", 1000);
  ros::Publisher joint2 = n.advertise<std_msgs::Float64>("/robot_arm/joint2_position_controller/command", 1000);
  ros::Publisher joint3 = n.advertise<std_msgs::Float64>("/robot_arm/joint3_position_controller/command", 1000);

   ros::Rate loop_rate(1000);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  std_msgs::Float64 angle;
  int choice;
  bool flag = 1;
  while (flag)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    cout << "1)Direct Kinematics" << endl;
    cout << "2)Inverse Kinematics" << endl;
    cout << "0)Exit" << endl;
    cout << "Insert your choice:";
    cin >> choice;
    switch(choice){
      case 1:
      {
        double q1,q2,q3;
        cout << "DIRECT KINEMATICS" << endl;
        cout << "Insert joint values" << endl;
        cout << "q1:";
        cin >> q1;
        cout << "q2:";
        cin >> q2;
        cout << "q3:";
        cin >> q3;
        std_msgs::String msg;
        angle.data = q1;
        joint1.publish(angle);
        angle.data = q2;
        joint2.publish(angle);
        angle.data = q3;
        joint3.publish(angle);
        cout << "published!" << endl;
        break;
      }
      case 2:
      {
        double x,y,z;
        cout << "INVERSE KINEMATICS" << endl;
        cout << "Insert the desired coordinates:" << endl;
        cout << "X:";
        cin >> x;
        cout << "Y:";
        cin >> y;
        cout << "Z:";
        cin >> z;
        double coords[3] = {x,y,z};
        if(checkPointInsideWorkspace(coords)){
          cout << "ok" << endl;
        }else{
          cout << endl << "ERROR: Coordinates outside the workspace" <<  endl << endl;
        }


        break;
      }
      case 0:
      {
        flag = 0;
        break;
      }
    }
    
  }

  return 0;
}
