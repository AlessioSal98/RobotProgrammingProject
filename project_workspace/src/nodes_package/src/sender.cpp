#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "Robot_arm.cpp"

using namespace std;
using namespace Eigen;

void publishJointValues(Vector3d q,ros::Publisher joint1,ros::Publisher joint2,ros::Publisher joint3){
  std_msgs::Float64 angle;
  angle.data = q(0);
  joint1.publish(angle);
  angle.data = q(1);
  joint2.publish(angle);
  angle.data = q(2);
  joint3.publish(angle);
  cout << "published!" << endl;
}

void directKinematics(Robot3R r,ros::Publisher joint1,ros::Publisher joint2,ros::Publisher joint3){
  Vector3d q;
  cout << "DIRECT KINEMATICS" << endl;
  cout << "Insert joint values" << endl;
  cout << "q1:";
  cin >> q(0);
  cout << "q2:";
  cin >> q(1);
  cout << "q3:";
  cin >> q(2);
  cout << "End Effector coordinates:" << r.directKinematics(q) << endl;
  publishJointValues(q,joint1,joint2,joint3);
}

void inverseKinematics(Robot3R r,ros::Publisher joint1,ros::Publisher joint2,ros::Publisher joint3,bool analytical){
  Vector3d coords;
  cout << "INVERSE KINEMATICS" << endl;
  cout << "Insert the desired coordinates:" << endl;
  cout << "X:";
  cin >> coords(0);
  cout << "Y:";
  cin >> coords(1);
  cout << "Z:";
  cin >> coords(2);
  if(r.checkPointInsideWorkspace(coords)){
    Vector3d q;
    if(analytical){
      Matrix<double, 4, 3> qsolutions;
      r.analyticalInverseKinematics(coords,qsolutions);
      bool exit=false;
      while(!exit)
      {
        int sol;
        cout << "Founded solutions:" << endl << qsolutions << endl;
        cout << "Choose the solution to publish (1-4) or press 0 to exit:";
        cin >> sol;
        switch(sol){
          case 1:
          {
            q << qsolutions(0), qsolutions(4), qsolutions(8);
            cout << "Publishing the solution " << sol << ":  [" << q(0) << "," << q(1) << "," << q(2) << "]" << endl;
            publishJointValues(q,joint1,joint2,joint3);
            break;
          }
          case 2:
          {
            q << qsolutions(1), qsolutions(5), qsolutions(9);
            cout << "Publishing the solution " << sol << ":  [" << q(0) << "," << q(1) << "," << q(2) << "]" << endl;
            publishJointValues(q,joint1,joint2,joint3);
            break;
          }
          case 3:
          {
            q << qsolutions(2) , qsolutions(6) , qsolutions(10);
            cout << "Publishing the solution " << sol << ":  [" << q(0) << "," << q(1) << "," << q(2) << "]" << endl;
            publishJointValues(q,joint1,joint2,joint3);
            break;
          }
          case 4:
          {
            q << qsolutions(3) , qsolutions(7) , qsolutions(11);
            cout << "Publishing the solution " << sol << ":  [" << q(0) << "," << q(1) << "," << q(2) << "]" << endl;
            publishJointValues(q,joint1,joint2,joint3);
            break;
          }
          case 0:
          {
            exit = true;
            break;
          }
        }
      }
    }
    else{
      Vector3d q0;
      q0 << 0,0,0;
      q = r.gradientMethod(coords,q0,0.01,0.001);
      publishJointValues(q,joint1,joint2,joint3);
    }
  }else{
    cout << endl << "ERROR: Coordinates outside the workspace" <<  endl << endl;
  }
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
   
   Vector3d origin_;
   origin_ << 0,0,2;
   Robot3R r(2,1,1,origin_);
   Vector3d q;
   q << 0,0,0;
   Matrix<double, 3, 3> j = r.jacobian(q);

  int count = 0;
  std_msgs::Float64 angle;
  int choice;
  bool flag = 1;
  while (flag)
  {
    cout << "1)Direct Kinematics" << endl;
    cout << "2)Inverse Kinematics (Analytical)" << endl;
    cout << "3)Inverse Kinematics (Numerical)" << endl;
    cout << "0)Exit" << endl;
    cout << "Insert your choice:";
    cin >> choice;
    switch(choice){
      case 1:
      {
        directKinematics(r,joint1,joint2,joint3);
        break;
      }
      case 2:
      {
        inverseKinematics(r,joint1,joint2,joint3,true);
        break;
      }
      case 3:
      {
        inverseKinematics(r,joint1,joint2,joint3,false);
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
