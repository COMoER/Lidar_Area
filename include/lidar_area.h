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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <mutex>

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZ> LidarPointCloud;

#define INIT_THRE 20
#define FILTER_THRE_HIGH 200000
#define FILTER_THRE_LOW 100000
#define K 0.1

class lidar_area{
private:
    uint16_t cols;//赛场被划分为几列
    uint16_t rows;//赛场被划分为几行
    uint16_t channels;

    const Vector2f x_threshold;
    const Vector2f y_threshold;
    const Vector2f z_threshold;

    float vol_size;

    LidarPointCloud** grids;

    pcl::PassThrough<PointXYZ> pass;

    MatrixXf T;


    ros::Publisher* _pub;
    ros::Subscriber* _sub;
    ros::NodeHandle* _lidar_n;

    uint64_t cloud_num;

    mutex lock;

    int getAreaOrder(int &i,int &j,int &k,const float x,const float y,const float z);
    void random_delete();
    void add_points(const LidarPointCloud &);
    void callback(const PointCloud2::ConstPtr &);
public:
    lidar_area(ros::NodeHandle &n,float vol_size,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high);
    int start(const string &lidar_topic = "/livox/lidar");
    ~lidar_area();



};
#endif //SRC_LIDAR_AREA_H
