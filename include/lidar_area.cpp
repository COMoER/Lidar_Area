//
// Created by sjtu on 2021/3/1.
//
#include "lidar_area.h"
#include "common.h"
using namespace pcl;



lidar_area::lidar_area(ros::NodeHandle &n,float vs,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high):
x_threshold(x_low,x_high),y_threshold(y_low,y_high),z_threshold(z_low,z_high),vol_size(vs),T(4,4)
{
    _lidar_n = &n;
    _sub = nullptr;
    _pub = nullptr;
    cloud_num = 0;
    cols = ceil((x_high-x_low)/vol_size);
    rows = ceil((y_high-y_low)/vol_size);
    channels = ceil((z_high-z_low)/vol_size);

    //TODO:transform from camera to world position

    string path;
    vector<float> extrinsic;
    if(!_lidar_n->getParam("Extrinsic",path))exit(1);
    ROS_INFO("Extrinsic path dir: %s",path.c_str());

    getExtrinsic(path,extrinsic);

    T << extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3],
          extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7],
          extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11],
          0.,0.,0.,1.;

    grids = new LidarPointCloud*[rows*cols];
    for(int i=0;i<rows*cols;++i) grids[i] = new LidarPointCloud;

    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_low,x_high);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_low,y_high);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_low,z_high);
    pass.setFilterLimitsNegative(false);




}
lidar_area::~lidar_area() {
    delete _pub;
    delete _sub;
    for(int i=0;i<rows*cols;++i) delete grids[i];
    delete grids;
}


int lidar_area::getAreaOrder(int &i,int &j,int &k,const float x,const float y,const float z) {

    i = floor((x-x_threshold[0])/vol_size);
    j = floor((y-y_threshold[0])/vol_size);

    return 0;

}
void lidar_area::callback(const PointCloud2::ConstPtr &data) {
    LidarPointCloud cloud_nan,cloud,cloud_filter,cloud_t;
    pcl::fromROSMsg(*data,cloud_nan);
    vector<int> index;
    pcl::removeNaNFromPointCloud(cloud_nan,cloud,index);

    pcl::transformPointCloud<PointXYZ>(cloud,cloud_t,T);

    pass.setInputCloud(cloud_t.makeShared());

    pass.filter(cloud_filter);

    uint64_t points_num = 0;

    for(int i=0;i<rows*cols;++i) points_num += grids[i]->size();

    if(points_num > FILTER_THRE_HIGH)
    {
        random_delete();
    }
    add_points(cloud_filter);
    ++cloud_num;


}
int lidar_area::start(const string &lidar_topic)
{
    assert(_lidar_n != nullptr);

    _pub = new ros::Publisher;
    _sub = new ros::Subscriber;


    *_pub = _lidar_n->advertise<sensor_msgs::PointCloud2>("livox_pc",1);
    *_sub = _lidar_n->subscribe(lidar_topic,10,&lidar_area::callback,this);

    ROS_INFO("Successfully Start!");

}

void lidar_area::random_delete() {

}

void lidar_area::add_points(const LidarPointCloud &cloud) {

}
