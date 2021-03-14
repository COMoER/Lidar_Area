//
// Created by sjtu on 2021/3/6.
//
#include "detect.hpp"
#include "common.h"

lcca::lcca(NodeHandle &n, const string &camera_topic)
{
    _lcca_n = &n;
    camera_sub = new Subscriber;
    pub = new Publisher;
    *camera_sub = _lcca_n->subscribe<sensor_msgs::Image>(camera_topic,10,&lcca::callback_camera,this);
    *pub = _lcca_n->advertise<std_msgs::Float32MultiArray>("camera_position",10);

    cv::namedWindow("camera",cv::WINDOW_NORMAL);
    K = cv::Mat::zeros(3,3,CV_64FC1);
    string path;
    if(!_lcca_n->getParam("Intrinsic",path))exit(1);
    vector<float> intrinsic;
    vector<float> distortion;
    getIntrinsic(path,intrinsic);
    getDistortion(path,distortion);

    K.at<double>(0, 0) = intrinsic[0];
    K.at<double>(0, 2) = intrinsic[2];
    K.at<double>(1, 1) = intrinsic[4];
    K.at<double>(1, 2) = intrinsic[5];

    // set radial distortion and tangential distortion
    C = cv::Mat::zeros(5,1,CV_64FC1);

    C.at<double>(0, 0) = distortion[0];
    C.at<double>(1, 0) = distortion[1];
    C.at<double>(2, 0) = distortion[2];
    C.at<double>(3, 0) = distortion[3];
    C.at<double>(4, 0) = distortion[4];


    info = new apriltag_detection_info_t();
    info->fx = K.at<double>(0,0);
    info->fy = K.at<double>(1,1);
    info->cx = K.at<double>(0,2);
    info->cy = K.at<double>(1,2);
    info->tagsize = 0.460;


    tf = tag25h9_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    spin_flag = false;

}


void lcca::callback_camera(const sensor_msgs::Image::ConstPtr &data) {
    if(spin_flag)
    {
        pub->publish(msg);
        return;
    }
    cv_bridge::CvImageConstPtr imgptr = cv_bridge::toCvCopy(data,data->encoding);
    cv::Mat img_raw = imgptr->image;
    cv::imshow("camera",img_raw);

    auto key = cv::waitKey(80);

    if (key == (0xFF & 'q'))
    {
        cv::destroyWindow("camera");
        ROS_INFO("Camera_position node will send position matrix continuously until you terminate whole program!");
        spin_flag = true;
    }
    if (key == (0xFF & 'o'))
    {
        cv::Mat img;
        cv::undistort(img_raw,img,K,C);
        cv::Mat gray;
        cv::cvtColor(img,gray,CV_BGR2GRAY);
        image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };
        zarray_t *detections = apriltag_detector_detect(td,&im);
        ROS_INFO("%d tags detected!",zarray_size(detections));
        if(zarray_size(detections) != 1)
        {
            ROS_ERROR("Please check the num of tags! Need 1!");
            Exit();
            return;
        }

        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);
        for(int i = 0;i<4;++i){
            cv::circle(img,cv::Point2d(det->p[i][0],det->p[i][1]),1,cv::Scalar(0,255,255));
            cv::imshow("camera",img);
            cv::waitKey(0);
        }


        info->det = det;


        apriltag_pose_t pose;
        double err = estimate_tag_pose(&*info, &pose);
        auto R = pose.R->data;
        auto t = pose.t->data;

        msg.clear();
        for(int i = 0;i<9;++i)
        {
            msg.push_back(R[i]);
            if((i+1)%3) msg.push_back(t[i/3]);
        }


        ROS_INFO("Location error is %.03f",err);
        zarray_destroy(detections);
        ROS_INFO("FINISH! camera_pose is gotten!");


    }


}


void lcca::Exit()
{
//    cv::destroyAllWindows();
    ros::shutdown();
}
