#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>
#include <iostream>

int main(int argc, char** argv){
	ros::init(argc, argv, "circle_example_node");
	
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd",100);
	
	ros::Time start_time = ros::Time::now();
	
	quadrotor_msgs::PositionCommand cmd;
	double angularz = M_PI / 16;
	unsigned int seq= 0 ;
	double time_dur,x,y,vx,vy,ax,ay;
	double yaw = M_PI / 2.0,yaw_dot = 0;
	ros::Time time_now;
	cmd.header.frame_id = "world";
	cmd.trajectory_id = 1;
	cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
	cmd.kx[0] = 0;
	cmd.kx[1] = 0;
	cmd.kx[2] = 0;
	cmd.kv[0] = 0;
	cmd.kv[1] = 0;
	cmd.kv[2] = 0;
	
	double radius = 5.0;
	
	ros::Rate rate(100);
	while(ros::ok()){
		time_now = ros::Time::now();
		cmd.header.seq = seq;
		cmd.header.stamp = time_now;
		
		time_dur = (time_now - start_time).toSec();
		
		x = radius * cos(angularz * time_dur - M_PI / 2.0);
		vx = -radius * angularz * sin(angularz * time_dur - M_PI / 2.0);
		ax = -radius * angularz * angularz * cos(angularz * time_dur - M_PI / 2.0);
		
		y = radius * sin(angularz * time_dur - M_PI / 2.0) + radius;
		vy = radius * angularz * cos(angularz * time_dur - M_PI / 2.0);
		ay = -radius * angularz * angularz * sin(angularz* time_dur - M_PI / 2.0);
		
		yaw =  angularz * (std::fmod(time_dur, (2 * M_PI / angularz)));
		
		ROS_INFO("x = %.2f, y = %.2f, dx = %.2f,dy = %.2f, yaw = %.2f, yawd = %.2f",x,y,ax,ay,yaw,yaw_dot);
		
		cmd.position.x = x;
		cmd.position.y = y;
 	   	cmd.position.z = 3;

		cmd.velocity.x = vx;
		cmd.velocity.y = vy;
		cmd.velocity.z = 0;

		cmd.acceleration.x = ax;
		cmd.acceleration.y = ay;
		cmd.acceleration.z = 0;
		
		cmd.yaw =  yaw;
		cmd.yaw_dot = angularz;
		
		cmd_pub.publish(cmd);
		rate.sleep();
		seq++;
	}
	
	return 0;
}
