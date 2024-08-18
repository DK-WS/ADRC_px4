#include <ros/ros.h>
#include <minisnap/planner.h>
#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv,"planner_function_test_node");
    planner myplanner;
    MatrixXd coeff = MatrixXd::Zero(1,24);
    coeff(0,0) = 1;
    coeff(0,1) = 1;
    coeff(0,2) = 1;
    coeff(0,3) = 1;
    Vector3d pos_ = myplanner.getPosPoly(coeff,0,1);
    Vector3d vel_ = myplanner.getVelPoly(coeff,0,1);
    Vector3d acc_ = myplanner.getAccPoly(coeff,0,1);
    Vector3d jerk_ = myplanner.getJerkPoly(coeff,0,1);
    cout << "pos:" << pos_ << endl;
    cout << "vel:" << vel_ << endl;
    cout << "acc:" << acc_ << endl;
    cout << "jerk:" << jerk_ << endl;
    ROS_INFO("null");
    return 0;
}