#include "airsim_node.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_node2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AirsimNode airsim_node(&nh, "90.0.0.196");
    ros::Rate rate(80);
    airsim_node.takeoff();
    while (ros::ok()) {
        airsim_node.move(0,0,0.5571,0,5);
        ros::spinOnce();
        rate.sleep();
    }
    airsim_node.land();
}