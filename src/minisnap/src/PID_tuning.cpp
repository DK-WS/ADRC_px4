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
	//double angularz = M_PI / 16;
	//unsigned int seq= 0 ;
	//double time_dur,x,y,vx,vy,ax,ay;
	//double yaw = M_PI / 2.0,yaw_dot = 0;
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
	
	ros::Rate rate(100);
    time_now = ros::Time::now();
		cmd.header.stamp = time_now;
		
		cmd.position.x = 0;
		cmd.position.y = 0;
 	   	cmd.position.z = 1;

		cmd.velocity.x = 0;
		cmd.velocity.y = 0;
		cmd.velocity.z = 0;

		cmd.acceleration.x = 0;
		cmd.acceleration.y = 0;
		cmd.acceleration.z = 0;
		
		cmd.yaw =  0;
		cmd.yaw_dot = 0;
    for(int i=0;i<500;i++){
        cmd_pub.publish(cmd);
        rate.sleep();
    }
    
	while(ros::ok()){
        time_now = ros::Time::now();
		cmd.header.stamp = time_now;
		
		cmd.position.x = 1;
		cmd.position.y = 0;
 	   	cmd.position.z = 1;

		cmd.velocity.x = 0;
		cmd.velocity.y = 0;
		cmd.velocity.z = 0;

		cmd.acceleration.x = 0;
		cmd.acceleration.y = 0;
		cmd.acceleration.z = 0;
		
		cmd.yaw =  0;
		cmd.yaw_dot = 0;
		
		cmd_pub.publish(cmd);
		rate.sleep();
	}
	
	return 0;
}
