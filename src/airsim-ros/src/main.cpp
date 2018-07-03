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

    string ip("192.168.1.100");
    ROS_INFO("Connetct IP: %s",ip.c_str());
    AirsimNode airsim_node(&nh, ip);
    ptr_airsim=&airsim_node;
    signal(SIGINT,sig);

    //************ 获取数据 *******************
    airsim_node.run();
    ROS_INFO("Start Airsim Control");
    
    //************* 控制部分 *****************
    airsim_node.takeoff();

    double target_height = 25.0;
    double d_throttle;

    PIDctrl pid_height;
    pid_height.init(0.1 , 0.0003 , 1.5 , 5);

    ros::Rate rate(30);
    while (airsim_node.RUNNING_FLAG) 
    {
        //pitch 负-前 roll 负-左  
        double d_height = target_height - airsim_node.barometer_data.vector.x;
        d_throttle = pid_height.calc(d_height);

        ROS_INFO("H: %f  D: %f" , d_height , d_throttle);   
        //airsim_node.move(-0.03 , 0 , d_throttle  , 0 , 5);  
 
        ros::spinOnce();
        rate.sleep();
    }
    airsim_node.land();
    
    ROS_INFO("Exit : Airsim Node");
}