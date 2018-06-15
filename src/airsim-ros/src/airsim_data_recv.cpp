#include "airsim_node.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <signal.h>
#include <thread>

using namespace std;

AirsimNode* ptr_airsim;

void sig(int isg_no){
    if (ptr_airsim==NULL)
    {
        ROS_ERROR("Get Exit Signal Error");
    }
    else{
        ptr_airsim->RUNNING_FLAG=0;
        ROS_INFO("Get Exit Signal");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AirsimNode airsim_node(&nh, "90.0.0.196");
    ptr_airsim=&airsim_node;
    signal(SIGINT,sig);
    airsim_node.run();
    ROS_INFO("Airsim Node Exit");
}