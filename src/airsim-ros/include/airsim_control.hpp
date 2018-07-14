#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "PID.h"
#include <iostream>
#include <string>
#include <gf_perception/Object.h>
#include <gf_perception/ObjectList.h>

class AirsimControl{
    public:
    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_magnetic;
    ros::Subscriber sub_baromrter;
    ros::Subscriber sub_object_front;
    ros::Subscriber sub_object_down;
    ros::Subscriber sub_object_circle;
    ros::Subscriber sub_err;

    void cb_imu(const sensor_msgs::Imu& msg);
    void cb_gps(const sensor_msgs::NavSatFix& msg);
    void cb_magnetic(const sensor_msgs::MagneticField& msg);
    void cb_barometer(const geometry_msgs::Vector3Stamped& msg);
    void cb_object_front(const gf_perception::ObjectList& msg);
    void cb_object_down(const gf_perception::ObjectList& msg);
    void cb_object_circle(const geometry_msgs::Vector3Stamped& msg);
    void cb_err(const std_msgs::Int16& msg);
    

    
    sensor_msgs::Imu msg_imu;
    sensor_msgs::NavSatFix msg_gps;
    sensor_msgs::MagneticField msg_magnetic;
    geometry_msgs::Vector3Stamped msg_barometer;
    geometry_msgs::Vector3Stamped msg_circle;
    gf_perception::ObjectList msg_objects_front;
    gf_perception::ObjectList msg_objects_down;
    gf_perception::Object object_front;
    gf_perception::Object object_down;
    int error_code;
    
    void run();
    int target_mode_count[11][2];
    int target_mode[11]; //0-未知,1-板子.2-停机坪
    int detect_num;       //1-10 ,11-二维码

    AirsimControl();
    ~AirsimControl(){;};
    
    msr::airlib::MultirotorRpcLibClient* control_client;
    bool takeoff(PIDctrl* pid);
    float initial_height;
    bool land();
    bool move(float pitch,float roll,float throttle,float yaw,float duration);
    void gostraight();


};