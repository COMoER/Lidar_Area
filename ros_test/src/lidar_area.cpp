//
// Created by sjtu on 2021/3/1.
//
#include "lidar_area.h"
using namespace pcl;
#define INIT_THRE 20
lidar_area::lidar_area(float vs,float x_low,float x_high,float y_low,float y_high,float h_high):
x_threshold(x_low,x_high),y_threshold(y_low,y_high),h_threshold(0,h_high),vol_size(vs)
{
    _lidar_n = nullptr;
    _pub = nullptr;
    init_num = 0;
    cols = ceil((x_high-x_low)/vol_size);
    rows = ceil((y_high-y_low)/vol_size);

    grids = new LidarPointCloud*[rows*cols];
    for(int i=0;i<rows*cols;++i) grids[i] = new LidarPointCloud;


}
lidar_area::~lidar_area() {
    delete _pub;
    delete _sub;
    for(int i=0;i<rows*cols;++i) delete grids[i];
    delete grids;
}


int lidar_area::getAreaOrder(int &i,int &j,const float x,const float y,const float z) {
    if(z > h_threshold[1] || z < h_threshold[0] || x < x_threshold[0] || x > x_threshold[1] || y <y_threshold[0] || y>y_threshold[1]) return -1;

    return 0;

}
void lidar_area::callback(const PointCloud2::ConstPtr &data) {
    pcl::PointCloud<PointXYZ> cloud;
    pcl::fromROSMsg(*data,cloud);
    


}
int lidar_area::start(const string &lidar_topic)
{
    assert(_lidar_n != nullptr);

    _pub = new ros::Publisher;
    _sub = new ros::Subscriber;


    *_pub = _lidar_n->advertise<sensor_msgs::PointCloud2>("livox_pc",1);
    *_sub = _lidar_n->subscribe(lidar_topic,10,&lidar_area::callback,this);

}
