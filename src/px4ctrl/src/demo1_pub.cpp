//发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布发布
#include "ros/ros.h"
// 1，包含头文件
#include "std_msgs/String.h"
#include <sstream>
//#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "std_msgs/Float32.h"

float t=0;


int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle nh;

    // 创建发布者对象
    ros::Publisher command_pub = nh.advertise<quadrotor_msgs::PositionCommand>("planning/pos_cmd", 100);

    // 定义消息对象并填充内容
    quadrotor_msgs::PositionCommand msg;
    msg.header.stamp = ros::Time::now(); // 设置时间戳为当前时间
    msg.position.x = 0 ;// 设置位置的x坐标为1.0
    msg.position.y = 0; // 设置位置的x坐标为1.0
    msg.position.z = 1; // 设置速度的z分量为-0.5
    // msg.yaw = 0.785; // 设置偏航角为0.0
    msg.yaw = -0.785;
    command_pub.publish(msg);

    //ros::Duration(5.0).sleep(); // 休眠2秒
    ros::Rate rate(100);
    // 发布消息
    while (ros::ok())
    {
        if (t<=400)
        {
            msg.header.stamp = ros::Time::now(); // 设置时间戳为当前时间
            msg.position.x = 0; // 设置位置的x坐标为1.0
            msg.position.y = 0; // 设置位置的x坐标为1.0
            msg.position.z = 5; // 设置速度的z分量为-0.5
        }

        
        if(t>314)
        {
            msg.header.stamp = ros::Time::now(); // 设置时间戳为当前时间
            msg.position.x = 5*std::cos(t/200); // 设置位置的x坐标为1.0
            msg.position.y = 5-5*std::sin(t/200); // 设置位置的x坐标为1.0
            msg.position.z = 5; // 设置速度的z分量为-0.5
        }
        
        t++;
        command_pub.publish(msg);
        rate.sleep();
 
    // ROS循环等待
        ros::spinOnce();//回调函数，建议添加
    }
    


    return 0;
}

/*
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    // 2，初始化节点
    ros::init(argc,argv,"ergou");
    // 3，创建节点句柄
    ros::NodeHandle nh;
    // 4，创建发布对象
    ros::Publisher pub= nh.advertise<std_msgs::String>("huati",10);
    ros::Publisher tra_generation_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);  //发布px4Ctrl控制指令
    // 5，编写发布逻辑
    std_msgs::String xiaoxi;
    //    设置频率
    ros::Rate rate(1);
    //    设置编号
    int count =0;

    while (ros::ok())
    {
        count++;
        std::stringstream ss;
        ss << "fk-->"<< count;
        xiaoxi.data= ss.str();

        pub.publish(xiaoxi);
        ROS_INFO("发布：%s",ss.str().c_str());

        rate.sleep();
        ros::spinOnce();//回调函数，建议添加
    }
    
    return 0;
}
*/