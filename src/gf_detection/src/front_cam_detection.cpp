#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "num_detect.hpp"
using namespace std;
using namespace cv;

Mat img_rgb, img_depth;
void cb_front_rgb(const sensor_msgs::ImageConstPtr& ptr_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(ptr_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_rgb = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;
}

void cb_front_depth(const sensor_msgs::ImageConstPtr& ptr_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(ptr_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_depth = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_front_rgb = it.subscribe("airsim/image/front/rgb", 1, &cb_front_rgb);
    image_transport::Subscriber sub_front_depth = it.subscribe("airsim/image/front/depth", 1, &cb_front_depth);
    
    ros::Rate rate(80);

    NumberDetect num_detect;
    while(ros::ok()){
        if (img_rgb.empty() ||img_depth.empty()){
            rate.sleep();
            ros::spinOnce();
            continue;
        }
        num_detect.detect(img_rgb,img_depth);
        Mat i;
        num_detect.colorExteact(img_rgb,i);
        imshow("a",img_rgb);
        waitKey(1);
         rate.sleep();
            ros::spinOnce();
    }
    return 0;
}