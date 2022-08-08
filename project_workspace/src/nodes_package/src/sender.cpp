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



/*
const double d = 2;
const double l2 = 1;
const double l3 = 1;
const double radius = 2;
const double origin[] = {0,0,d};
*/

/*
//Checks if the required cartesian point is inside the workspace of the robot arm
bool checkPointInsideWorkspace(double* coords){
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
*/

/*
//Utility function used to solve a linear system when computing inverse kinematics
Vector2d solveSystem(double cosq3,double q3, double q1, double px, double py, double pz){
  Matrix2d A;
  Vector2d b;
  Vector2d x;
  A << l2+l3*cosq3, -l3*sin(q3), l3*sin(q3), l2 + l3*cosq3;
  b << cos(q1)*px + sin(q1)*py, pz - d;
  x = A.colPivHouseholderQr().solve(b);
  return x;
}
*/

/*
//Function that computes the values of q with inverse kinematics in the analytical way, it computes all the solution but returns only the first one
bool analyticalInverseKinematics(double coords[],Matrix<double, 4, 3> &qsolutions){
  double px = coords[0];
  double py = coords[1];
  double pz = coords[2];
  bool singularity = false;
  double cosq3 = (pow(px,2)+pow(py,2)+pow(pz-d,2)-pow(l2,2)-pow(l3,2))/(2*l2*l3);
  if (cosq3<-1 || cosq3>1){
    singularity = true;
  }else{
    double sinq3 = sqrt(1-pow(cosq3,2));
    double q3pos = atan2(sinq3,cosq3);
    double q3neg = atan2(-sinq3,cosq3);
    if(pow(px,2)+pow(py,2)==0){
      singularity=true;
    }else{
      double q1pos = atan2(py,px);
      double q1neg = atan2(-py,-px);
      if ((pow(px,2)+pow(py,2))+pow(pz-d,2) == 0){
        singularity = true;
      }else{
        Vector2d xpp = solveSystem(cosq3,q3pos,q1pos,px,py,pz);
        Vector2d xpn = solveSystem(cosq3,q3pos,q1neg,px,py,pz);
        Vector2d xnp = solveSystem(cosq3,q3neg,q1pos,px,py,pz);
        Vector2d xnn = solveSystem(cosq3,q3neg,q1neg,px,py,pz);


        double q2pp = atan2(xpp(1),xpp(0));
        double q2pn = atan2(xpn(1),xpn(0));
        double q2np = atan2(xnp(1),xnp(0));
        double q2nn = atan2(xnn(1),xnn(0));

        //Matrix<double, 4, 3> qsolutions;
        qsolutions << q1pos,q2pp,q3pos , q1pos,q2pn,q3neg, q1neg,q2np,q3pos, q1neg,q2nn,q3neg;
      }
    }
  }
  return singularity;
}
*/


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

void directKinematics(ros::Publisher joint1,ros::Publisher joint2,ros::Publisher joint3){
  Vector3d q;
  cout << "DIRECT KINEMATICS" << endl;
  cout << "Insert joint values" << endl;
  cout << "q1:";
  cin >> q(0);
  cout << "q2:";
  cin >> q(1);
  cout << "q3:";
  cin >> q(2);
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
    cout << "1)Direct Kinematics" << endl;
    cout << "2)Inverse Kinematics (Analytical)" << endl;
    cout << "3)Inverse Kinematics (Numerical)" << endl;
    cout << "0)Exit" << endl;
    cout << "Insert your choice:";
    cin >> choice;
    switch(choice){
      case 1:
      {
        directKinematics(joint1,joint2,joint3);
        break;
      }
      case 2:
      {
        inverseKinematics(r,joint1,joint2,joint3,true);
        break;
      }
      case 3:
      {
        //inverseKinematics(r,joint1,joint2,joint3,false);
        Vector3d rd;
        Vector3d q0;
        rd << 0,2,2;
        q0 << 0,1.57,1.57;
        r.newtonMethod(rd,q0);
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
