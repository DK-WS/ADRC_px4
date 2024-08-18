#include <ros/ros.h>
#include <iostream>
#include <regex>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  bool takeoffComplete = false;
  // 初始化ROS节点
  ros::init(argc, argv, "input_recognition_node");
  ros::NodeHandle nh;
  ros::Publisher goalPoint_pub = nh.advertise<geometry_msgs::Point>("/goal_point",1);
  ros::Publisher takeoff_pub = nh.advertise<std_msgs::Bool>("/keyboard/takeoff_cmd",1);

  // 定义正则表达式模式
  std::regex takeoff_pattern("takeoff");
  std::regex takeland_pattern("takeland");
  std::regex quit_pattern("q");
  std::regex coordinate_pattern("([+-]?[0-9]+\\.?[0-9]*),([+-]?[0-9]+\\.?[0-9]*),([+-]?[0-9]+\\.?[0-9]*)");
  
  while(ros::ok()){
    // 获取终端输入字符串
    std::string input;
    std::cout << "请输入命令或坐标点 x,y,z(输入q以退出):";
    std::getline(std::cin, input);
    
    // 匹配输入内容
    std::smatch match;
    if (std::regex_match(input, match, takeoff_pattern))
    {
      takeoffComplete = true;
      ROS_INFO("识别到 takeoff 命令");
      // geometry_msgs::Point startPoint;
      // startPoint.x = 0;
      // startPoint.y = 0;
      // startPoint.z = 2;
      std_msgs::Bool takeoffcmd;
      takeoffcmd.data = true;
      takeoff_pub.publish(takeoffcmd);
      
    }
    else if (std::regex_match(input, match, takeland_pattern))
    {
      ROS_INFO("识别到 takeland 命令");
      if(!takeoffComplete){
        ROS_WARN("请先输入takeoff指令");
      }else{
        std_msgs::Bool takeoffcmd;
        takeoffcmd.data = false;
        takeoff_pub.publish(takeoffcmd);
        takeoffComplete = false;
      }
      // geometry_msgs::Point startPoint;
      // startPoint.x = 0;
      // startPoint.y = 0;
      // startPoint.z = 0;
      // goalPoint_pub.publish(startPoint);
    }
    else if (std::regex_match(input, match, coordinate_pattern))
    {
      if(!takeoffComplete){
        ROS_WARN("请先输入takeoff指令");
      }else{
        geometry_msgs::Point goalPoint;
        float x = std::stof(match[1].str());
        float y = std::stof(match[2].str());
        float z = std::stof(match[3].str());
        goalPoint.x = x;
        goalPoint.y = y;
        goalPoint.z = z;
        ROS_INFO("识别到坐标点： x = %.2f, y = %.2f, z = %.2f", x, y, z);
        goalPoint_pub.publish(goalPoint);
      }
    }
    else if(std::regex_match(input, match, quit_pattern))
    {
      break;
    }
    else
    {
      ROS_INFO("非法输入");
    }
  }
  
  return 0;
}