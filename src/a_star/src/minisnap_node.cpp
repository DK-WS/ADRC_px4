#include <ros/ros.h>
#include <minisnap/planner.h>

geometry_msgs::PoseStamped px4_pose;      //实际姿态数据
geometry_msgs::Point rec_goal_point;//3D nav goal 给出的目标点数据类型
geometry_msgs::PointStamped goal_point;//绘制目标点的数据类型
bool clicked = false;
bool start_plan = false;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pt){
    px4_pose = *pt;
    // std::cout << "px4_pose:  " << px4_pose << std::endl;
}
void clicked_cb(const geometry_msgs::Point::ConstPtr &point){
    clicked = true;
    rec_goal_point = *point;
    ROS_INFO("CLICKED");
}

int main(int argc, char** argv){
    ros::init(argc,argv,"trajec_node");
    planner myplanner;
	ros::NodeHandle nh;
    nav_msgs::Path real_path;
    geometry_msgs::PoseStamped pose;
    std::vector<quadrotor_msgs::PositionCommand> tra;
    std::vector<geometry_msgs::Point> goal_point_list;

	ros::Publisher tra_generation_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10); 
    ros::Publisher real_trajectory_pub = nh.advertise<nav_msgs::Path>("/real_trajectory", 10, true); 
    ros::Publisher goal_point_pub = nh.advertise<geometry_msgs::PointStamped>("/path_point", 10, true);
    ros::Publisher despath_pub = nh.advertise<nav_msgs::Path>("/desire_trajectory", 10, true);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber clickedpoint_sub = nh.subscribe<geometry_msgs::Point>("/astar_path_point", 1, clicked_cb);
    ros::Rate rate(100);  //设置轨迹发布频率

    myplanner.dot_num = 0;

    real_path.header.frame_id = "world";
    real_path.header.stamp = ros::Time::now();

    while(ros::ok()){
        if(!start_plan && clicked){
            clicked = false;
            if(rec_goal_point.z < -100){//接受到特殊点时开始规划轨迹，特殊点不计入目标点
                start_plan = true; 
                myplanner.time_everytraj.resize(goal_point_list.size());//目标点数量没有包含初始位置
                for(int i = 0;i < goal_point_list.size() ; i++){
                    myplanner.time_everytraj( i ) = 2;//分配每段轨迹时间
                }
                myplanner.dot_num = goal_point_list.size() + 1 ;//点的总数量等于目标点数量加1（初始点）
                myplanner.route.resize(myplanner.dot_num,3);
                myplanner.route(0,0) = px4_pose.pose.position.x;
                myplanner.route(0,1) = px4_pose.pose.position.y;
                myplanner.route(0,2) = px4_pose.pose.position.z;
                for(int i = 0; i < goal_point_list.size() ; i++){
                    myplanner.route(i+1, 0) = goal_point_list[i].x;
                    myplanner.route(i+1, 1) = goal_point_list[i].y;
                    myplanner.route(i+1, 2) = goal_point_list[i].z;
                }
                //std::cout << "route:" << myplanner.route << endl;
                //std::cout << "time:" << myplanner.time_everytraj << endl;
                myplanner.poly_coeff = myplanner.getcoeff();//获得轨迹的多项式矩阵
                //std::cout << "poly_coeff:" << endl << myplanner.poly_coeff << endl;
                tra = myplanner.get_trajectory();//将轨迹离散化得到控制命令
                myplanner.draw_desire_trajectory(despath_pub);//可视化期望轨迹
                std::cout << tra.size() << endl;
            }else{
                goal_point_list.push_back(rec_goal_point);
                goal_point.header.frame_id = "world";
                goal_point.header.stamp = ros::Time::now();
                goal_point.point.x = rec_goal_point.x;
                goal_point.point.y = rec_goal_point.y;
                goal_point.point.z = rec_goal_point.z;
                goal_point_pub.publish(goal_point);
            }
        }else if(start_plan && clicked){
            clicked = false;
        }
        if(start_plan){
            static int i = 0;
            if( i < tra.size()){
                tra_generation_pub.publish(tra[i]);
                i++;
            }else{
                i = 0;
                start_plan = false;
                goal_point_list.clear();
                rec_goal_point.z = 0;
                std::cout<< "publish complete" <<endl;
            }
            
        }
        //可视化真实轨迹
        pose.pose.position.x = px4_pose.pose.position.x;
        pose.pose.position.y = px4_pose.pose.position.y;
        pose.pose.position.z = px4_pose.pose.position.z;
        real_path.poses.push_back(pose);
        real_trajectory_pub.publish(real_path);

        rate.sleep();
        ros::spinOnce();
    }
	return 0;
}
