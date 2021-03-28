//
// Created by sjtu on 2021/3/5.
//
#include "lidar_area.h"

int main(int argc, char **argv)
{


    //ros_node初始化
    ros::init(argc,argv,"lidar_ros_area_node");
    //node管理者
    ros::NodeHandle n("lidar_ros_area_node");

    string lt;
    if(!n.getParam("lidar_topic",lt))
    {
        lt = "/livox/lidar";
        ROS_INFO("No receive! Lidar_topic sets to default %s",lt.c_str());
    }
    float xmin,xmax,ymin,ymax,zmin,zmax,vol_size;
    n.getParam("xmin",xmin);
    n.getParam("xmax",xmax);
    n.getParam("ymin",ymin);
    n.getParam("ymax",ymax);
    n.getParam("zmin",zmin);
    n.getParam("zmax",zmax);
    n.getParam("vol_size",vol_size);

    ROS_INFO("INIT");

    lidar_area la(n,vol_size,xmin,xmax,ymin,ymax,zmin,zmax);
    la.start(lt);
    ros::Rate r(50);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}
