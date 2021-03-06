//
// Created by sjtu on 2021/3/1.
//
#include "lidar_area.h"
using namespace pcl;

lidar_area::lidar_area(float vol_size,float x_low,float x_high,float y_low,float y_high,float h_high):
x_threshold(x_low,x_high),y_threshold(y_low,y_high),h_threshold(0,h_high)
{
    _lidar_n = nullptr;
    _pub = nullptr;
}
lidar_area::~lidar_area() {
    delete _pub;
    delete _sub;
}


int lidar_area::getAreaOrder(Vector2i &order, const Vector3f &p) {
    return 0;

}
void lidar_area::callback(const PointCloud2::ConstPtr &data) {
    pcl::PointCloud<PointXYZ> cloud;
    PCLPointCloud2 pc2_cloud;
    pcl_conversions::toPCL(*data,pc2_cloud);
    pcl::fromPCLPointCloud2(pc2_cloud,cloud);
    pcl::VoxelGrid<PointXYZ> vg;
    vg.getFilterLimits()
}
int lidar_area::start(const string &lidar_topic )
{
    assert(_lidar_n != nullptr);

    _pub = new ros::Publisher;
    _sub = new ros::Subscriber;


    *_pub = _lidar_n->advertise<sensor_msgs::PointCloud2>("livox_pc",1);
    *_sub = _lidar_n->subscribe(lidar_topic,10,&lidar_area::callback,this);



}
