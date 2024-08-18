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

#include "Astar.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

//分辨率、分辨率倒数、未知？？？
double _resolution, _inv_resolution, _cloud_margin;

//世界坐标系(单位m)下，整个点云地图的长、宽、高
double _x_size, _y_size, _z_size;    

//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长、宽、高
int _max_x_id, _max_y_id, _max_z_id;

//起点坐标 世界坐标系(单位m)
Vector3d _start_pt;

//地图三轴最小和最大尺寸 世界坐标系(单位m)
Vector3d _map_lower, _map_upper;

std::string _distance;
double _weight_a,_weight_b;


ros::Subscriber _map_sub;               //点云地图的接收者
ros::Subscriber _pts_sub;               //终点坐标的接收者
ros::Publisher  _grid_path_vis_pub;     //发布Astar找到的路径
ros::Publisher  _visited_nodes_vis_pub; //发布OpenList/CloseList的方格
ros::Publisher  _grid_map_vis_pub;      //发布点云地图
ros::Publisher _path_point_pub;

//Astar算法的对象指针
AstarPathFinder * _astar_path_finder     = NULL;

//标志位，确保先有地图再有终点坐标
bool _has_map   = false;

void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

//终点坐标的回调函数
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{    
    //安全性检查 
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;
    //目标点的三维值  世界坐标系
    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
}

//点云地图的回调函数
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    // if(_has_map ) return;

    //PCL点云格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    static pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    //ROS点云格式
    sensor_msgs::PointCloud2 map_vis;
    //将ROS格式转成PCL格式
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;   //世界坐标系的三维点
    if(!_has_map){
        _has_map = true;
        for (int idx = 0; idx < (int)cloud.points.size(); idx++)
        {    
            pt = cloud.points[idx];        

            // 通过世界坐标系的三维点云坐标 设置 一维点云数据格式中的障碍物
            _astar_path_finder->setObs(pt.x, pt.y, pt.z);
            
            // 仅仅为了显示
            Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = cor_round(2);
            cloud_vis.points.push_back(pt);
        }
    } 

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;
    cout << cloud_vis.points.size()<<endl;
    //PCL格式-->ROS格式
    pcl::toROSMsg(cloud_vis, map_vis);
    //ros::Rate(2).sleep();
    map_vis.header.frame_id = "world";
    // while(!_grid_map_vis_pub.getNumSubscribers()>1){
    //     ROS_INFO("waiting for subscribe topic point_map");
    // }
    _grid_map_vis_pub.publish(map_vis);
    //std::cout << "map:" << map_vis << endl;
    ROS_INFO("point_map published");
}

//发布Astar路径
void visGridPath( vector<Vector3d> nodes)
{   
    visualization_msgs::Marker node_vis; 
    geometry_msgs::Point point;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
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
    

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);

        point.x = coord(0);
        point.y = coord(1);
        point.z = coord(2);
        _path_point_pub.publish(point);
    }
    point.x = 0;
    point.y = 0;
    point.z = -200;
    _path_point_pub.publish(point);
    _grid_path_vis_pub.publish(node_vis);
}

//发布OpenList/CloseList的方格
void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}

//路径查找，主要调用Astar的部分接口函数
void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    //调用A星寻路算法
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //通过A星算法得到路径点集和close集合
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //可视化路径和close集合
    visGridPath (grid_path);
    visVisitedNode(visited_nodes);

    //重置Astar算法，方便下次掉用
    _astar_path_finder->resetUsedGrids();

}

//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_demo");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("point_map", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes",1);
    _path_point_pub               = nh.advertise<geometry_msgs::Point>("/astar_path_point",1);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    
    nh.param("heuristic/distance",_distance,string("euclidean"));
    nh.param("weight/a",_weight_a,1.0);
    nh.param("weight/b",_weight_b,1.0);

    //地图三轴最小和最大尺寸 世界坐标系(单位m)
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    //分辨率倒数
    _inv_resolution = 1.0 / _resolution;
    
    //栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽、长、高
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    
    //构建astar指针对象
    _astar_path_finder  = new AstarPathFinder(_distance,_weight_a,_weight_b);
    
    //初始化点云地图
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper,_max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    //删除astar指针对象
    delete _astar_path_finder;
    return 0;
}

