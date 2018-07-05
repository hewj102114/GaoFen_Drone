#include "airsim_node.hpp"
#include "PID.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <signal.h>
#include <thread>

using namespace std;

AirsimNode *ptr_airsim;

void sig(int isg_no)
{
    if (ptr_airsim == NULL)
    {
        ROS_ERROR("Get Exit Signal Error");
    }
    else
    {
        ptr_airsim->RUNNING_FLAG = 0;
        ROS_INFO("Command : Get Exit Signal");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string ip("192.168.1.100");
    ROS_INFO("Connetct IP: %s", ip.c_str());
    AirsimNode airsim_node(&nh, ip);
    ptr_airsim = &airsim_node;
    signal(SIGINT, sig);

    //************ 获取数据 *******************
    airsim_node.run();
    ROS_INFO("Start Airsim Control");

    //************* 控制部分 *****************
    
    //状态变量
    int detect_num = 0;        //1-10 ,11-二维码
    int detect_state = 0;      //0-寻找,1-RGB图像检测,2-RGB+深度图像检测,3-圆圈检测，4-穿越/降落
    int target_mode[11] = {0}; //0-未知,1-板子.2-停机坪
    target_node[0] = 2;
    target_node[1] = 2;
    target_node[2] = 2;
    target_node[10] = 2;

    int lost_detect_rgb=0;

    //airsim_node.takeoff();

    ros::Rate rate(30);
    while (airsim_node.RUNNING_FLAG)
    {
        //数字
        if (detect_num < 11)
        {
            //板子未知类型 探索
            if (target_mode[detect_num] == 0) 
            {
                ;
            }
            //障碍圈
            if (target_mode[detect_num] == 1) 
            {
                ;
            }
            //停机坪
            if (target_mode[detect_num] == 2) 
            {
                //finding
                if(detect_state==0){
                    
                }
            }


        }
        //二维码
        else
        {
            ;
        }

        ros::spinOnce();
        rate.sleep();
    }
    // airsim_node.land();

    int exit_flag_sum = accumulate(airsim_node.exit_ready_flag, airsim_node.exit_ready_flag + 7, (int)0);
    while (exit_flag_sum < 5)
    {
        sleep(1);
        ROS_INFO("Wait For Thread Exit");
        exit_flag_sum = accumulate(airsim_node.exit_ready_flag, airsim_node.exit_ready_flag + 7, (int)0);
    }
    ROS_INFO("Exit : Airsim Node");
}