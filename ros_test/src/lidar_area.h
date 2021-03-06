//
// Created by sjtu on 2021/3/1.
//

#ifndef SRC_LIDAR_AREA_H
#define SRC_LIDAR_AREA_H

#include <Eigen/Core>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;

typedef vector<Vector3d> PointCloud;

class lidar_area{
private:
    uint16_t cols;//赛场被划分为几列
    uint16_t rows;//赛场被划分为几行

    const Vector2f x_threshold;
    const Vector2f y_threshold;
    const Vector2f h_threshold;


    ros::Publisher* _pub;
    ros::Subscriber* _sub;
    ros::NodeHandle* _lidar_n;

    mutex lock;

    int getAreaOrder(Vector2i &,const Vector3f &);
    void callback(const PointCloud2::ConstPtr &);
public:
    lidar_area(float vol_size,float x_low,float x_high,float y_low,float y_high,float h_high);
    int start(const string &lidar_topic = "/livox/lidar");
    ~lidar_area();
    void setNode(ros::NodeHandle &n){_lidar_n = &n;}



};
#endif //SRC_LIDAR_AREA_H
