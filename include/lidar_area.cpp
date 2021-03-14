//
// Created by sjtu on 2021/3/1.
//
#include "lidar_area.h"
#include "common.h"

using namespace pcl;

lidar_area::lidar_area(ros::NodeHandle &n,float vs,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high):
x_threshold(x_low,x_high),y_threshold(y_low,y_high),z_threshold(z_low,z_high),vol_size(vs),T(4,4),c_T(4,4),l_T(4,4),DEBUG(false)
{
    _lidar_n = &n;
    _sub = nullptr;
    _pub = nullptr;

    srand(unsigned(time(NULL)));

    cols = ceil((x_high-x_low)/vol_size);
    rows = ceil((y_high-y_low)/vol_size);
    channels = ceil((z_high-z_low)/vol_size);

    x_ex = x_low + float(rows)*vol_size - x_high;
    y_ex = y_low + float(cols)*vol_size - y_high;
    z_ex = z_low + float(channels)*vol_size - z_high;

    //TODO:transform from camera to world position

    pose_init = false;
    pose_get = false;

    string pose_topic;
    if(!_lidar_n->getParam("pose_topic",pose_topic))pose_topic = "/camera_pose";
    ROS_INFO("Pose topic is %s.",pose_topic.c_str());
    pose_sub = new ros::Subscriber;
    *pose_sub = _lidar_n->subscribe(pose_topic,10,&lidar_area::pose_callback,this);

    string path;
    vector<float> extrinsic;
    if(!_lidar_n->getParam("Extrinsic",path))exit(1);
    ROS_INFO("Extrinsic path dir: %s",path.c_str());

    getExtrinsic(path,extrinsic);


    l_T << extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3],
          extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7],
          extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11],
          0.,0.,0.,1.;

    T = l_T;

    grids = new LidarPointCloud*[rows*cols*channels];
    for(int i=0;i<rows*cols*channels;++i) {
        //message init
        msg.size.push_back(0);
        msg.center_z.push_back(0.);
        msg.header.seq = 0;
        grids[i] = new LidarPointCloud;
    }
    n.getParam("debug",DEBUG);

    if(DEBUG)
    {
        ROS_INFO("DEBUG MODE ...");
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("cloud"));
        viewer->setBackgroundColor(0,0,0);
        viewer->addPointCloud<PointXYZ>(sum_cloud.makeShared());
    }

    points_num = 0;

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


void lidar_area::getAreaOrder(int &i,int &j,int &k,const float x,const float y,const float z) {

    i = floor((x-x_threshold[0])/vol_size);
    j = floor((y-y_threshold[0])/vol_size);
    k = floor((z-z_threshold[0])/vol_size);

}

void lidar_area::callback(const PointCloud2::ConstPtr &data) {
    if(get_pose()) return;
    LidarPointCloud cloud_nan,cloud,cloud_filter,cloud_t;
    pcl::fromROSMsg(*data,cloud_nan);
    vector<int> index;
    pcl::removeNaNFromPointCloud(cloud_nan,cloud,index);

    pcl::transformPointCloud<PointXYZ>(cloud,cloud_t,T);

    pass.setInputCloud(cloud_t.makeShared());

    pass.filter(cloud_filter);

    if(points_num > FILTER_THRE_HIGH)
    {
        random_delete();
    }
    add_points(cloud_filter);



    if(points_num >= FILTER_THRE_LOW)
    {
        ++msg.header.seq;
        _pub->publish(msg);
    }

    if(DEBUG)
    {
        sum_cloud.clear();
        for(int i = 0;i < rows*cols*channels;++i) sum_cloud += *grids[i];
        //viewer->updatePointCloud(sum_cloud.makeShared());
        viewer->spinOnce();
    }

}
int lidar_area::start(const string &lidar_topic)
{
    assert(_lidar_n != nullptr);

    _pub = new ros::Publisher;
    _sub = new ros::Subscriber;


    *_pub = _lidar_n->advertise<lidar_ros_area::Grid>("livox_pc",1);//TODO:a msg to define
    *_sub = _lidar_n->subscribe(lidar_topic,10,&lidar_area::callback,this);

    ROS_INFO("Successfully Start!");
    ROS_INFO("Waiting for lidar...");
    ROS_INFO("Waiting for camera pose...");

}

void lidar_area::random_delete() {
    float rate = 1;
    int msg_index;
    float sum_z;
    for(int i = 0;i<rows;++i)
    for(int j = 0;j<cols;++j)
    for(int k = 0;k<channels;++k)
    {
        if(i == rows-1) rate *= vol_size/x_ex;
        if(j == cols-1) rate *= vol_size/y_ex;
        if(k == channels -1) rate *= vol_size/z_ex;
        LidarPointCloud *cloud = index(i,j,k);
        double r = 1.0/(1.0+exp(-K*cloud->size()*rate))-0.5;    //delete rate
        MatrixXd e = Eigen::MatrixXd::Random(cloud->size(),1).array().abs(); //(0,1) uniform
        auto begin = cloud->begin();
        int size = cloud->size();
        for(int index = 0;index < size;++index)
        {
            if(e(i,0) <= r){
                msg_index = i*rows*channels+j*channels+k;
                sum_z = msg.center_z[msg_index]*msg.size[msg_index]-(*cloud)[i].z;
                msg.center_z[msg_index] = sum_z/(--msg.size[msg_index]);
                --points_num;
                cloud->erase(begin+index);
            }

        }

    }


}

void lidar_area::add_points(const LidarPointCloud &cloud) {
    int msg_index;
    float sum_z;
    int size = cloud.size();
    int whole_size = points_num;
    MatrixXd e;
    if(RANDOM_ADD && whole_size >= FILTER_THRE_LOW)
    {
        e = Eigen::MatrixXd::Random(size,1).array().abs(); //(0,1) uniform
    }
    double r = 1.9 - 1.8/(1.0+exp(-K_2*(whole_size-FILTER_THRE_LOW)));    //increasing rate
    for(int n=0;n < size;++n)
    {
        if(RANDOM_ADD && whole_size >= FILTER_THRE_LOW) if(e(n,0) >= r) continue;
        auto point = cloud[n];
        int i,j,k;
        getAreaOrder(i,j,k,point.x,point.y,point.z);
        index(i,j,k) ->push_back(point);
        msg_index = i*rows*channels+j*channels+k;
        sum_z = msg.center_z[msg_index]*msg.size[msg_index]+point.z;
        msg.center_z[msg_index] = sum_z/(++msg.size[msg_index]);
        ++points_num;
    }
}

void lidar_area::pose_callback(const std_msgs::Float32MultiArrayConstPtr &data) {
    if(pose_init) return;
    vector<float> pose = data->data;
    c_T << pose[0], pose[1], pose[2], pose[3],
            pose[4], pose[5], pose[6], pose[7],
            pose[8], pose[9], pose[10], pose[11],
            0.,0.,0.,1.;
    ROS_INFO("Camera pose successfully init!");
    pose_init = true;
}
int lidar_area::get_pose() {
    if(pose_get) return 0;
    if(!pose_init) return 1;
    T = c_T*l_T;
    pose_get = true;
    ROS_INFO("lidar successfully init!");
    return 0;
}

