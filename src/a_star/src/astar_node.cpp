#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include "a_star/Astar_searcher.h"
#include "a_star/backward.hpp"
#include "minisnap/planner.h"

using namespace std;
using namespace Eigen;


bool goal_set = false;
bool start_plan = false;
bool takeoffCmd = false;
string pcdPath;
geometry_msgs::PoseStamped px4_pose;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ> cloudMap_vis;    
AstarPathFinder *pathfinderPtr = NULL;

//分辨率、分辨率倒数、暂时无用
double _resolution, _inv_resolution, _cloud_margin;

//世界坐标系(单位m)下，整个点云地图的长、宽、高
double _x_size, _y_size, _z_size;    

//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长、宽、高
int _max_x_id, _max_y_id, _max_z_id;

//起点坐标 世界坐标系(单位m)
Vector3d _start_pt,_goal_pt;

//地图三轴最小和最大坐标 世界坐标系(单位m)
Vector3d _map_lower, _map_upper;

//h的计算方式
string distanceType;
//g和h的权重
double _weight_a,_weight_b;

void Load_map(){
    pcl::io::loadPCDFile(pcdPath,cloudMap);

    //if( (int)cloudMap.points.size() == 0 ) return;
    assert((int)cloudMap.points.size() != 0);
}

void Init_Astar_map(AstarPathFinder *pathfinderPtr){
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    pathfinderPtr -> initGridMap(_resolution,_map_lower,_map_upper,_max_x_id,_max_y_id,_max_z_id);

    pcl::PointXYZ point;
    for(int i = 0;i < (int)cloudMap.points.size();i++){
        point = cloudMap.points[i];
        pathfinderPtr->setObs(point.x,point.y,point.z);
    }
    int obs_count = 0;
    for(int i=0;i<(int)pathfinderPtr->data.size();i++){
        if (pathfinderPtr -> data[i]){
            obs_count ++;
        }
    }
    ROS_INFO("obstacle count : %d", obs_count);
}

void visualize_map(AstarPathFinder *pathfinderPtr){
    pcl::PointXYZ point;
    for(int i = 0;i < (int)cloudMap.points.size();i++){
        point = cloudMap.points[i];
        
        Vector3d cor_round = pathfinderPtr -> coordRounding(Vector3d(point.x, point.y, point.z));
        point.x = cor_round(0);
        point.y = cor_round(1);
        point.z = cor_round(2);
        cloudMap_vis.points.push_back(point);
    }
    cout << cloudMap.points.size() << "  " << cloudMap_vis.points.size() << endl;
    cloudMap_vis.width = cloudMap_vis.points.size();
    cloudMap_vis.height = 1;
    cloudMap_vis.is_dense = true;

    pcl::toROSMsg(cloudMap_vis, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
}

void init_marker(visualization_msgs::Marker* marker){
    // 设置消息类型为LINE_STRIP
    marker->type = visualization_msgs::Marker::LINE_STRIP;
    // 设置坐标系
    marker->header.frame_id = "world";
    marker->header.stamp = ros::Time::now();
    marker->ns = "";
    marker->id = 0;
    marker->action = visualization_msgs::Marker::ADD;
    marker->pose.orientation.w = 1.0;
    // 设置线条的宽度
    marker->scale.x = 0.02;
    marker->scale.y = 0.02;
    marker->scale.z = 0.02;
    // 设置线条的颜色
    marker->color.r = 0.0;
    marker->color.g = 1.0;
    marker->color.b = 1.0;
    marker->color.a = 1.0;

    // 清空列表的点
    marker->points.clear();
}

void visGridPath( vector<Vector3d> nodes, ros::Publisher _grid_path_vis_pub)
{   
    visualization_msgs::Marker node_vis; 
    geometry_msgs::Point point;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;
    

    node_vis.scale.x = _resolution/5;
    node_vis.scale.y = _resolution/5;
    node_vis.scale.z = _resolution/5;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void goalPointCallback(const geometry_msgs::Point::ConstPtr &point){
    if(point->x >= _map_lower(0) && point->x <= _map_upper(0) &&
       point->y >= _map_lower(1) && point->y <= _map_upper(1) &&
       point->z >= _map_lower(2) && point->z <= _map_upper(2))
    {
        _goal_pt << point->x,
                     point->y,
                     point->z;
        goal_set = true;
    }
    else
    {
        ROS_INFO("goal_point data is illegal");
    }
    
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose){
    px4_pose = *pose;
    //ROS_INFO("px4 pose get");
}

void takeoffCallback(const std_msgs::Bool::ConstPtr &cmd){
    takeoffCmd = cmd ->data;
}

int main(int argc, char** argv){

    int cmd_state = 0;
    int landing_state = 0;

    ros::init(argc,argv,"astar_node");
    ros::NodeHandle nh;
    planner myplanner;

    visualization_msgs::Marker real_trajectory_marker;
    vector<quadrotor_msgs::PositionCommand> tra;

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_map",1);//可视化点云地图
    ros::Publisher _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("/astar_path_marker", 1);//可视化路径点
    ros::Publisher tra_generation_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);  //发布px4Ctrl控制指令
    //ros::Publisher despath_pub = nh.advertise<nav_msgs::Path>("/desire_trajectory", 10, true);
    ros::Publisher desTrajectory_pub = nh.advertise<visualization_msgs::Marker>("/desire_trajectory",1,true);//可视化minisnap规划的轨迹
    ros::Publisher realTrajectory_pub = nh.advertise<visualization_msgs::Marker>("/real_trajectory",1,true);//可视化minisnap规划的轨迹

    ros::Subscriber goalPoint_sub = nh.subscribe("/goal_point",1, goalPointCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber takeoffCmd_sub = nh.subscribe<std_msgs::Bool>("/keyboard/takeoff_cmd", 1, takeoffCallback);

    nh.param("pcd_path", pcdPath, string("/home/ws/minisnap-1.0/src/a_star/pcd/map.pcd"));
    nh.param("heuristic/distance",distanceType,string("euclidean"));
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 10.0);
    nh.param("map/y_size",        _y_size, 10.0);
    nh.param("map/z_size",        _z_size, 4.0 );
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    nh.param("weight/a",_weight_a,1.0);
    nh.param("weight/b",_weight_b,1.0);

    pathfinderPtr = new AstarPathFinder(distanceType,_weight_a,_weight_b);
    
    Load_map();
    Init_Astar_map(pathfinderPtr);
    visualize_map(pathfinderPtr);

    while(map_pub.getNumSubscribers()<1);
    ros::Duration(3).sleep();       //延时，防止rviz接收不到地图
    map_pub.publish(globalMap_pcd);
    init_marker(&real_trajectory_marker);
    while(ros::ok()){
        switch (cmd_state)
        {
        case 0:
            if(!start_plan && takeoffCmd){
                // cmd_state = 1;
                myplanner.dot_num = 2;
                myplanner.time_everytraj.resize(1);
                myplanner.time_everytraj(0) = 10;
                myplanner.route.resize(2, 3);
                myplanner.route(0,0) = px4_pose.pose.position.x;
                myplanner.route(0,1) = px4_pose.pose.position.y;
                myplanner.route(0,2) = px4_pose.pose.position.z;

                myplanner.route(1,0) = px4_pose.pose.position.x;
                myplanner.route(1,1) = px4_pose.pose.position.y;
                myplanner.route(1,2) = 2;
                myplanner.poly_coeff = myplanner.getcoeff();
                cout << myplanner.poly_coeff << endl;
                tra = myplanner.get_trajectory(); //离散化轨迹
                myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                start_plan = true;
            }
            if(start_plan){
                static int i = 0;
                if( i < (int)tra.size()){
                    tra_generation_pub.publish(tra[i]);
                    geometry_msgs::Point temp_point;
                    temp_point.x = px4_pose.pose.position.x;
                    temp_point.y = px4_pose.pose.position.y;
                    temp_point.z = px4_pose.pose.position.z;
                    real_trajectory_marker.points.push_back(temp_point);
                    realTrajectory_pub.publish(real_trajectory_marker);
                    i++;
                }else{
                    i = 0;
                    start_plan = false;
                    std::cout<< "takeoff complete" <<endl;
                    cmd_state = 1;
                }
            }
            break;
        
        case 1:
            if(!start_plan && !takeoffCmd){
                cmd_state = 2;
                break;
            }
            if(!start_plan && goal_set){
                goal_set = false;
                if(pathfinderPtr->isReachable(_goal_pt)){
                    pathfinderPtr -> AstarGraphSearch(Vector3d(px4_pose.pose.position.x,px4_pose.pose.position.y,px4_pose.pose.position.z), _goal_pt);
                    //通过A星算法得到路径点集和close集合
                    auto grid_path     = pathfinderPtr->getPath();
                    auto visited_nodes = pathfinderPtr->getVisitedNodes();

                    //可视化路径点
                    visGridPath(grid_path, _grid_path_vis_pub);
                    //重置Astar算法，方便下次调用
                    pathfinderPtr->resetUsedGrids();

                    //A*算法结束，开始用minisnap算法规划轨迹
                    myplanner.dot_num = ((int)grid_path.size()) + 1 ;
                    myplanner.time_everytraj.resize(myplanner.dot_num - 1);
                    myplanner.route.resize(myplanner.dot_num, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;
                    Vector3d node;
                    geometry_msgs::Point point;
                    for(int i = 0; i < ((int)grid_path.size()); i++){
                        node = grid_path[i];
                        myplanner.time_everytraj(i) = 1;//每一格的时间
                        myplanner.route(i+1, 0) = node(0);
                        myplanner.route(i+1, 1) = node(1);
                        myplanner.route(i+1, 2) = node(2);
                    }
                    myplanner.poly_coeff = myplanner.getcoeff();
                    tra = myplanner.get_trajectory(); //离散化轨迹
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    ROS_INFO("the end point is not reachable");
                }
            }
            if(start_plan){
                goal_set = false;
                static int i = 0;
                if( i < (int)tra.size()){
                    tra_generation_pub.publish(tra[i]);
                    geometry_msgs::Point temp_point;
                    temp_point.x = px4_pose.pose.position.x;
                    temp_point.y = px4_pose.pose.position.y;
                    temp_point.z = px4_pose.pose.position.z;
                    real_trajectory_marker.points.push_back(temp_point);
                    realTrajectory_pub.publish(real_trajectory_marker);
                    i++;
                }else{
                    i = 0;
                    start_plan = false;
                    std::cout<< "publish complete" <<endl;
                }
            }
            break;
        
        case 2:
            switch (landing_state)
            {
            case 0:
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = px4_pose.pose.position.x;
                    myplanner.route(1,1) = px4_pose.pose.position.y;
                    myplanner.route(1,2) = 5;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); //离散化轨迹
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        std::cout<< "landing takeoff complete" <<endl;
                        landing_state = 1;
                    }
                }
                break;
            
            case 1:
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = 0;
                    myplanner.route(1,1) = 0;
                    myplanner.route(1,2) = px4_pose.pose.position.z;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); //离散化轨迹
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        std::cout<< "landing up complete" <<endl;
                        landing_state = 2;
                    }
                }
                break;
            case 2:
                if(!start_plan){
                    myplanner.dot_num = 2;
                    myplanner.time_everytraj.resize(1);
                    myplanner.time_everytraj(0) = 10;
                    myplanner.route.resize(2, 3);
                    myplanner.route(0,0) = px4_pose.pose.position.x;
                    myplanner.route(0,1) = px4_pose.pose.position.y;
                    myplanner.route(0,2) = px4_pose.pose.position.z;

                    myplanner.route(1,0) = 0;
                    myplanner.route(1,1) = 0;
                    myplanner.route(1,2) = 0;
                    myplanner.poly_coeff = myplanner.getcoeff();
                    cout << myplanner.poly_coeff << endl;
                    tra = myplanner.get_trajectory(); //离散化轨迹
                    myplanner.draw_desire_trajectory_marker(desTrajectory_pub);
                    start_plan = true;
                }else{
                    static int i = 0;
                    if( i < (int)tra.size()){
                        tra_generation_pub.publish(tra[i]);
                        geometry_msgs::Point temp_point;
                        temp_point.x = px4_pose.pose.position.x;
                        temp_point.y = px4_pose.pose.position.y;
                        temp_point.z = px4_pose.pose.position.z;
                        real_trajectory_marker.points.push_back(temp_point);
                        realTrajectory_pub.publish(real_trajectory_marker);
                        i++;
                    }else{
                        i = 0;
                        start_plan = false;
                        landing_state = 0;
                        cmd_state = 0;
                        goal_set = false;
                        std::cout<< "landing complete" <<endl;
                    }
                }
                break;
            default:
                break;
            }
            
            break;
        }
        ros::spinOnce();
        ros::Rate(100).sleep();
    }
    delete pathfinderPtr;
    return 0;
}
