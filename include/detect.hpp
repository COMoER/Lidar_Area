//
// Created by sjtu on 2021/3/4.
//

#ifndef SRC_DETECT_HPP
#define SRC_DETECT_HPP

#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>

//opencv and apriltag
#include <opencv2/opencv.hpp>
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/common/getopt.h"
#include <apriltag/apriltag_pose.h>


#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

#include <iostream>

using namespace ros;

using namespace std;


using namespace Eigen;

class lcca
{
private:


    NodeHandle* _lcca_n;
    Subscriber* camera_sub;
    Publisher* pub;


    cv::Mat K;
    cv::Mat C;
    apriltag_detector_t *td;
    apriltag_detection_info_t* info;
    apriltag_family_t *tf;

    vector<float> msg;

    bool spin_flag;



    void callback_camera(const sensor_msgs::Image::ConstPtr&data);

    void Exit();
public:
    lcca(NodeHandle &n,const string& camera_topic);
    ~lcca(){delete camera_sub;
        delete pub;
        apriltag_detector_destroy(td);
        tag25h9_destroy(tf);
        delete info;
    }
};

#endif //SRC_DETECT_HPP
