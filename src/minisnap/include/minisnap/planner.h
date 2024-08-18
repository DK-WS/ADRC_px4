// #ifndef ROS_NODE.H
// #define ROS_NODE.H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "minisnap/trajectory_generator.h"
#include <visualization_msgs/Marker.h>
using namespace std;
using namespace Eigen;
#define SNAP 4
#define JERK 3


class  TrajectoryGeneratorTool;
class planner
{
    private:
        int poly_coeff_num;
        int mode = SNAP;
        nav_msgs::Path real_path;
        
    public:
        planner()                               //构造函数；
        {
            
        }
        ros::NodeHandle n ;
        int  dot_num ;      //保存约束点个数       
        Eigen::MatrixXd route;          //矩阵保存约束点
        Eigen::VectorXd time_everytraj ;               //向量保存轨迹时间
        Eigen::MatrixXd poly_coeff;     //   系数矩阵     
        
        // friend class TrajectoryGeneratorTool;   //  友元类
        void getparam(void);    //获取参数
        Eigen::MatrixXd getcoeff( void);
        Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        Eigen::Vector3d getVelPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        Eigen::Vector3d getAccPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        Eigen::Vector3d getJerkPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        std::vector<quadrotor_msgs::PositionCommand> get_trajectory(void);
        void tra_publish(void);
        void draw_desire_trajectory(ros::Publisher despath_pub);
        void draw_desire_trajectory_marker(ros::Publisher despath_pub);
};

// #endif