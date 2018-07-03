#include "airsim_node.hpp"
#include "PID.h"
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
        ROS_INFO("Command : Get Exit Signal");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    

    union fffa{
        float num;
        uchar data[4];
    }numb;
    numb.data[0]=0;
    numb.data[1]=0;
    numb.data[2]=128;
    numb.data[3]=127;
    cout<<"dddd  "<<numb.num<<endl;

    string ip("192.168.1.20");
    ROS_INFO("Connetct IP: %s",ip.c_str());
    AirsimNode airsim_node(&nh, ip);
    ptr_airsim=&airsim_node;
    signal(SIGINT,sig);

    //************ 获取数据 *******************
    airsim_node.run();
    ROS_INFO("Start Airsim Control");
    
    //************* 控制部分 *****************
    //******  初始化

    // airsim_node.takeoff();
    double target_height=10.0;
    double pre_throttle=0.5571;
    PIDctrl pid_height;
    pid_height.init(0.00006,0,0.05,0.1);

    ros::Rate rate(30);
    while (airsim_node.RUNNING_FLAG) {
        //pitch 负-前 roll 负-左 throttle 0.5571
        sleep(1);
        // double d_height=target_height-airsim_node.gps_data.altitude;

        // double d_throttle= pid_height.calc(d_height);
        // ROS_INFO("H: %f  D: %f",d_height,d_throttle);
        // airsim_node.move(-0,0,pre_throttle + d_throttle,0,5);
        // pre_throttle=pre_throttle + d_throttle;
        // ros::spinOnce();
        // rate.sleep();
    }
    // airsim_node.land();
    
    int exit_flag_sum=accumulate(airsim_node.exit_ready_flag,airsim_node.exit_ready_flag+7,(int)0);
    while(exit_flag_sum<7){
        sleep(1);
        ROS_INFO("Wait For Thread Exit");
        exit_flag_sum=accumulate(airsim_node.exit_ready_flag,airsim_node.exit_ready_flag+7,(int)0);
    }
    ROS_INFO("Exit : Airsim Node");
}