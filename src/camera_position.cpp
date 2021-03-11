//
// Created by sjtu on 2021/3/3.
//

#include "detect.hpp"
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"camera_position");

    string ct;
    string lt;
    ros::NodeHandle n("camera_position");
    if(!n.getParam("camera_topic",ct)) ct = "/zed2/zed_node/left/image_rect_color";
    lcca calibrator(n,ct);
    ros::spin();

    return 0;
}

