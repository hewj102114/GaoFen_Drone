#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <iostream>
#include <string>

class AirsimNode{
    public:
    AirsimNode(ros::NodeHandle* _pnh,const std::string& _ip);
    ~AirsimNode();
    std::string ip_adress;

    ros::NodeHandle* pnh;
    ros::Publisher pub_imu;
    ros::Publisher pub_magnetic;
    ros::Publisher pub_gps;
    ros::Publisher pub_barometer;
    image_transport::Publisher pub_image_front_rgb;
    image_transport::Publisher pub_image_front_depth;
    image_transport::Publisher pub_image_down_rgb;

    sensor_msgs::Imu imu_data;
    sensor_msgs::MagneticField magnetic_data;
    sensor_msgs::NavSatFix gps_data;
    geometry_msgs::Vector3Stamped barometer_data;

    msr::airlib::MultirotorRpcLibClient* control_client;
    bool RUNNING_FLAG,EXIT_FLAG;

    void getAirsimData(msr::airlib::MultirotorRpcLibClient* client);
    void getImuData(msr::airlib::MultirotorRpcLibClient* client);
    void getMagneticData(msr::airlib::MultirotorRpcLibClient* client);
    void getGPSData(msr::airlib::MultirotorRpcLibClient* client);
    void getBarometerData(msr::airlib::MultirotorRpcLibClient* client);
    void getImageFrontRgbData(msr::airlib::MultirotorRpcLibClient* client);
    void getImageFrontDepthData(msr::airlib::MultirotorRpcLibClient* client);
    void getImageDownRgbData(msr::airlib::MultirotorRpcLibClient* client);
    void getAllImageData(msr::airlib::MultirotorRpcLibClient* client);

    void connect();
    bool takeoff();
    bool land();
    bool move(float pitch,float roll,float throttle,float yaw,float duration);
    bool hover();

    void run();
    void flightDataThread();
    void imageFrontRgbThread();
    void imageFrontDepthThread();
    void imageDownRgbThread();

};