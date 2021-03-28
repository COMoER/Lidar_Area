//
// Created by sjtu on 2021/3/1.
//

#ifndef SRC_LIDAR_AREA_H
#define SRC_LIDAR_AREA_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ctime>

#include <lidar_ros_area/Grid.h>

#include <thread>


using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZ> LidarPointCloud;

#define INIT_THRE 20
#define FILTER_THRE_HIGH 200000
#define FILTER_THRE_LOW 100000
#define K 0.01
#define RANDOM_ADD true
#define K_2 0.00003

class lidar_area{
private:
    int cols;//赛场被划分为几列
    int rows;//赛场被划分为几行
    int channels;

    const Vector2f x_threshold;
    const Vector2f y_threshold;
    const Vector2f z_threshold;


    float x_ex;
    float y_ex;
    float z_ex;

    float vol_size;

    int points_num;

    LidarPointCloud** grids;

    pcl::PassThrough<PointXYZ> pass_x;
    pcl::PassThrough<PointXYZ> pass_y;
    pcl::PassThrough<PointXYZ> pass_z;

    ros::Publisher _cloud_pub;

    LidarPointCloud grids_points;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color;

    MatrixXf l_T;
    MatrixXf c_T;
    MatrixXf T;

    bool pose_init;
    bool pose_get;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  //定义窗口共享指针

    lidar_ros_area::Grid msg;

    bool DEBUG;


    LidarPointCloud sum_cloud;

    ros::Publisher* _pub;
    ros::Subscriber* _sub;
    ros::Subscriber* pose_sub;
    ros::NodeHandle* _lidar_n;

    void getAreaOrder(int &i,int &j,int &k,float x,float y,float z);
    void random_delete();
    void add_points(const LidarPointCloud &);
    void callback(const PointCloud2::ConstPtr &);
    void pose_callback(const std_msgs::Float32MultiArrayConstPtr &);
    int get_pose();
    LidarPointCloud* index(const int i,const int j,const int k){
        return grids[i*cols*channels+j*channels+k];
    }
public:
    lidar_area(ros::NodeHandle &n,float vol_size,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high);
    int start(const string &lidar_topic = "/livox/lidar");

    ~lidar_area();



};
#endif //SRC_LIDAR_AREA_H
