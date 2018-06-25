#include "airsim_node.hpp"
#include "PID.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_node2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AirsimNode airsim_node(&nh, "192.168.1.20");
    ros::Rate rate(30);
    airsim_node.takeoff();

    double target_height=15.0;

    double pre_throttle=0.5571;

    PIDctrl pid_height;
    pid_height.init(0.0001,0,0,1);


    while (ros::ok()) {
        //pitch 负-前 roll 负-左 throttle 0.5571
        double d_height=airsim_node.gps_data.altitude-target_height;

        double d_throttle= pid_height.calc(d_height);
        ROS_INFO("H: %f  D: %f",airsim_node.gps_data.altitude,d_throttle);
        airsim_node.move(-0,0,pre_throttle + d_throttle,0,5);
        pre_throttle=pre_throttle + d_throttle;
        ros::spinOnce();
        rate.sleep();
    }
    airsim_node.land();
}