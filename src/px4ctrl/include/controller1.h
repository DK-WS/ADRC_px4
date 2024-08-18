/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <queue>
#include "ros/ros.h"
#include "ROScallback.h"
#include <Eigen/Dense>
double fhan(double x1, double x2, double r, double h);
double sat(double x, double delta);
double fal(double x, double alpha, double delta) ;
void euler_from_quaternion(double x, double y, double z, double w, 
                           double &roll, double &pitch, double &yaw);
Eigen::Vector3d inertial_to_body_frame(const Eigen::Vector3d& target_point, double roll, double pitch, double yaw);
void compute_uv(const Eigen::Vector3d& c_3d, double fx, double fy, double u0, double v0, 
                double image_plane_width, double image_plane_height, 
                double& u, double& v, double& flag);
struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;
    double head_rate;

    Desired_State_t(){};

    Desired_State_t(Odom_Data_t &odom)
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0),
          head_rate(0){};
};

struct Controller_Output_t
{

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;
    Eigen::Quaterniond orientation;
  
    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]
    double roll_rate;
	double pitch_rate;
	double yaw_rate;
    double normalized_thrust;
    // Collective mass normalized thrust
    double thrust;

    Eigen::Vector3d des_v_real;
};

struct SO3_Controller_Output_t
{
	Eigen::Matrix3d Rdes;
	Eigen::Vector3d Fdes;
	double net_force;
};

class LinearControl
{
public:
    LinearControl(Parameter_t &);

    Eigen::Vector3d int_e_v;
    Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
    double Kyaw;
    double dt;
    Eigen::MatrixXd A, B, Q, N,R,K;
   
    quadrotor_msgs::Px4ctrlDebug
    update(
    const Desired_State_t& des, 
	const Odom_Data_t& odom,
    const Imu_Data_t &imu,  
	Controller_Output_t& u, 
	SO3_Controller_Output_t& u_so3
    );
    //////////////////////////////////////////////
    quadrotor_msgs::Px4ctrlDebug
    ADRC_Control(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu, 
        Controller_Output_t &u);
    //////////////////////////////////////////////
    quadrotor_msgs::Px4ctrlDebug
    calculateControl(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu, 
        Controller_Output_t &u);

    quadrotor_msgs::Px4ctrlDebug
    DLQR_Control(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu, 
        Controller_Output_t &u);
    
    
    quadrotor_msgs::Px4ctrlDebug
    update_alg1(const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu,
        Controller_Output_t &u);

    Controller_Output_t computeNominalReferenceInputs(
    const Desired_State_t& reference_state,
    const Odom_Data_t& attitude_estimate) const;
    Eigen::Vector3d computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const; 

    Eigen::Quaterniond computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) const;
    bool estimateThrustModel(const Eigen::Vector3d &est_v);
    bool almostZero(const double value) const;
	bool almostZeroThrust(const double thrust_value) const;
    void resetThrustMapping(void);
    void DLQR(const Eigen::MatrixXd &A, 
    const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R, const double tolerance,
    const int max_num_iteration, Eigen::MatrixXd *ptr_K);

    void DARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                    const Eigen::MatrixXd &N, Eigen::MatrixXd *K, const double eps);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Parameter_t param_;
    quadrotor_msgs::Px4ctrlDebug debug_msg_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;
    static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	static constexpr double kAlmostZeroThrustThreshold_ = 0.01;

    // Thrust-accel mapping params
    const double rho2_ = 0.998; // do not change
    double thr2acc_;
    double P_;

    void normalizeWithGrad(const Eigen::Vector3d &x,
        const Eigen::Vector3d &xd,
        Eigen::Vector3d &xNor,
        Eigen::Vector3d &xNord) const;

    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
    double fromQuaternion2yaw(Eigen::Quaterniond q);
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


// double fhan(double x1, double x2, double r, double h) {
//     double d = r * h * h;
//     double a0 = h * x2;
//     double y = x1 + a0;
//     double a1 = sqrt(d * (d + 8 * abs(y)));
//     double a2 = a0 + copysign(1, y) * (a1 - d) / 2.0;
//     double a = (a0 + y) * (copysign(1, y + d) - copysign(1, y - d)) / 2.0 + a2 * (1 - (copysign(1, y + d) - copysign(1, y - d)) / 2.0);
//     double fhan = -r * (a / d) * (copysign(1, y + d) - copysign(1, y - d)) / 2.0 - r * copysign(1, a) * (1 - (copysign(1, a + d) - copysign(1, a - d)) / 2.0);
//     return fhan;
// }
// double sat(double x, double delta) {
//     if (std::abs(x) < delta) {
//         return x / delta;
//     } else {
//         return std::copysign(1.0, x);
//     }
// }

// double fal(double x, double alpha, double delta) {
//     if (std::abs(x) < delta) {
//         return x / std::pow(delta, 1 - alpha);
//     } else {
//         return std::pow(std::abs(x), alpha) * std::copysign(1.0, x);
//     }
// }


class ADRCController {
private:
    //参数区，这11个就是需要用户整定的参数
    /****************TD**********/
    double r ,//快速跟踪因子
           h ;//滤波因子,系统调用步长

    /**************ESO**********/
    double  b       ,//系统系数
            delta   ,//delta为fal（e，alpha，delta）函数的线性区间宽度
            aa      ,
            belta01 ,//扩张状态观测器反馈增益1
            belta02 ,//扩张状态观测器反馈增益2
            belta03 ;//扩张状态观测器反馈增益3
        
    /**************NLSEF*******/
    double alpha1 =0.7,//
           alpha2 =1.5,//
           belta1 ,//跟踪输入信号增益
           belta2 ;//跟踪微分信号增益
    //内部中间变量
    double x2,x1,z1,z2,z3,u,u0,e,e1,e2;
    // 控制器参数


    // double y; // 实际输出
    // double u; // 控制输出
    // double e; // 跟踪误差
    // double z1; // 内层控制状态变量
    // double z2; // 外层控制状态变量

    // // ADRC参数
    // double h; // 内层滤波系数
    // double beta; // 外层参数
    // double rho; // 扰动估计参数

public:
    // 构造函数，初始化参数
    ADRCController(double r_,double h_,double aa_,double b_,double delta_,double belta1_,double belta2_)
    {
        h=h_;
        r=r_;
        aa=aa_;
        b=b_;
        delta=delta_;
        belta1=belta1_;
        belta2=belta2_;
        
        belta01 =3*aa;//扩张状态观测器反馈增益1
        belta02 = 3*aa*aa;//扩张状态观测器反馈增益2
        belta03 = aa*aa*aa ;//扩张状态观测器反馈增益3
        

        x1=0;
        x2=0;
        z1=0;
        z2=0;
        z3=0;
        u=0;
        u0=0;
        e=0;
        e1=0;
        e2=0;

    }

    // ADRC控制器计算
    double calculateControl(double v,double y,double des_v,double dom_v) 
    {
        x1 =  h*x2 + x1;
        x2 = x2 + h*fhan(x1-v,x2,r,h);
        /******************************ESO***************************************/
        // e = z1 - y;
        // z1 = z1 + h*(z2-belta01*e);
        // z2 = z2 + h*(z3-belta02*fal(e,0.5,delta)+b*u);
        // z3 = z3 + h*(-belta03*fal(e,0.25,delta));
        e = z1 - y;
        z1 = z1 + h*(z2-belta01*e);
        z2 = z2 + h*(z3-belta02*e+b*u);
        z3 = z3 + h*(-belta03*e);
        // ROS_INFO("de:%f,%f,%f",belta01,belta02,belta03);
        /******************************NLSEF*************************************/
        e1 = x1 - z1;
        e2 = x2 - z2;
        
        // u0 = belta1*fal(e1,0.7,delta) + belta2*fal(e2,1.5,delta);
        
       //自建
        // x1 =  h*x2 + x1;
        // x2 = x2 + h*fhan(x1-v,x2,r,h);

        // u=belta1*(v-y)+belta2*(des_v-dom_v);
        u0=belta1*(x1-z1)+belta2*(x2-z2);
        u = u0 - z3/b;


        return u;
    }
};



#endif