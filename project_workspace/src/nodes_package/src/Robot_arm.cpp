#include <iostream>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include <cmath>

using namespace std;
using namespace Eigen;

 class Robot3R {       
  public:             
    double d;      
    double l2;
    double l3; 
    double radius;
    double origin[];

    Robot3R(double d_,double l2_,double l3_,double* origin_){
      d=d_;
      l2=l2_;
      l3=l3_;
      radius=l2+l3;
      origin[0] = origin_[0];
      origin[1] = origin_[1];
      origin[2] = origin_[2];
    };

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

  //Utility function used to solve a linear system when computing inverse kinematics
  Vector2d solveSystem(double cosq3,double q3, double q1, double px, double py, double pz){
    Matrix2d A;
    Vector2d b;
    Vector2d x;
    double d = this->d;
    double l2 = this->l2;
    double l3 = this->l3;

    A << l2+l3*cosq3, -l3*sin(q3), l3*sin(q3), l2 + l3*cosq3;
    b << cos(q1)*px + sin(q1)*py, pz - d;
    x = A.colPivHouseholderQr().solve(b);
    return x;
  }
  //Function that computes the values of q with inverse kinematics in the analytical way, it computes all the solution but returns only the first one
  bool analyticalInverseKinematics(double coords[],Matrix<double, 4, 3> &qsolutions){
    double px = coords[0];
    double py = coords[1];
    double pz = coords[2];
    bool singularity = false;
    d = this->d;
    l2 = this->l2;
    l3 = this->l3;

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

          qsolutions << q1pos,q2pp,q3pos , q1pos,q2pn,q3neg, q1neg,q2np,q3pos, q1neg,q2nn,q3neg;
          
        }
      }
    }
    return singularity;
  }
};