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
    else ROS_INFO("Lidar_topic sets to %s",lt.c_str());
    lidar_area la(n,0.05,10.7,28.0,0.0,14.0,0,3.0);
    la.start(lt);
    ros::Rate r(50);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}
