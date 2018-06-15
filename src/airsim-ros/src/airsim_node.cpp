#include "airsim_node.hpp"
#include "common/CommonStructs.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
using namespace std;

#define DATA_FREQ 100000

AirsimNode::AirsimNode(ros::NodeHandle* _pnh, const std::string& _ip) {
    pnh = _pnh;
    pub_imu = pnh->advertise<sensor_msgs::Imu>("airsim/imu", 1);
    pub_magnetic =
        pnh->advertise<sensor_msgs::MagneticField>("airsim/magnetic", 1);
    pub_gps = pnh->advertise<sensor_msgs::NavSatFix>("airsim/gps", 1);
    pub_barometer =
        pnh->advertise<geometry_msgs::Vector3Stamped>("airsim/barometer", 1);

    image_transport::ImageTransport it(*pnh);
    pub_image_front_rgb = it.advertise("airsim/image/front/rgb", 1);
    ;
    pub_image_front_depth = it.advertise("airsim/image/front/depth", 1);
    pub_image_down_rgb = it.advertise("airsim/image/down/rgb", 1);

    ip_adress=_ip;
    control_client = new msr::airlib::MultirotorRpcLibClient(_ip);
    control_client->confirmConnection();

    RUNNING_FLAG=0;
    EXIT_FLAG=0;
}

AirsimNode::~AirsimNode() {  delete control_client;}

void AirsimNode::getAirsimData(msr::airlib::MultirotorRpcLibClient* client) {
    getImuData(client);
    getMagneticData(client);
    getGPSData(client);
    getBarometerData(client);
}

void AirsimNode::getImageFrontRgbData(msr::airlib::MultirotorRpcLibClient* client) {
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::Scene)
        };

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_rgb=cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);


    cv_bridge::CvImage msg_front_rgb;
    msg_front_rgb.header.stamp = ros::Time::now();
    msg_front_rgb.header.frame_id = "image";
    msg_front_rgb.image = img_front_rgb;
    msg_front_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_front_rgb.publish(msg_front_rgb.toImageMsg());


}

void AirsimNode::getImageFrontDepthData(msr::airlib::MultirotorRpcLibClient* client){
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::DepthPerspective, true)
        };

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_depth(response[0].height,response[0].width,CV_32FC1,response[0].image_data_float.data());

    cv_bridge::CvImage msg_front_depth;
    msg_front_depth.header.stamp = ros::Time::now();
    msg_front_depth.header.frame_id = "image";
    msg_front_depth.image = img_front_depth;
    msg_front_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    pub_image_front_depth.publish(msg_front_depth.toImageMsg());
}

void AirsimNode::getImageDownRgbData(msr::airlib::MultirotorRpcLibClient* client){
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

   
    std::vector<ImageRequest> request = {
        ImageRequest(3, ImageType::Scene)
        };

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_down_rgb=cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);

    cv_bridge::CvImage msg_down_rgb;
    msg_down_rgb.header.stamp = ros::Time::now();
    msg_down_rgb.header.frame_id = "image";
    msg_down_rgb.image = img_down_rgb;
    msg_down_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_down_rgb.publish(msg_down_rgb.toImageMsg());
}

void AirsimNode::getAllImageData(msr::airlib::MultirotorRpcLibClient* client){
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::Scene),
        ImageRequest(1, ImageType::Scene),
        ImageRequest(0, ImageType::DepthPlanner, true)
        };

    std::vector<ImageResponse> response = client->simGetImages(request);
   
    cv::Mat img_front_rgb=cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat img_down_rgb=cv::imdecode(response[1].image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat img_front_depth(response[2].height,response[2].width,CV_32FC1,response[2].image_data_float.data());

    cv_bridge::CvImage msg_front_rgb;
    msg_front_rgb.header.stamp = ros::Time::now();
    msg_front_rgb.header.frame_id = "image";
    msg_front_rgb.image = img_front_rgb;
    msg_front_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_front_rgb.publish(msg_front_rgb.toImageMsg());

    cv_bridge::CvImage msg_down_rgb;
    msg_down_rgb.header.stamp = ros::Time::now();
    msg_down_rgb.header.frame_id = "image";
    msg_down_rgb.image = img_down_rgb;
    msg_down_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_down_rgb.publish(msg_down_rgb.toImageMsg());

    cv_bridge::CvImage msg_front_depth;
    msg_front_depth.header.stamp = ros::Time::now();
    msg_front_depth.header.frame_id = "image";
    msg_front_depth.image = img_front_depth;
    msg_front_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    pub_image_front_depth.publish(msg_front_depth.toImageMsg());

    
}

void AirsimNode::getImuData(msr::airlib::MultirotorRpcLibClient* client) {
    msr::airlib::ImuData airsim_img_data;
    airsim_img_data = client->getImudata(1.0 / DATA_FREQ);
    imu_data.header.frame_id = "base_link";
    imu_data.header.stamp = ros::Time::now();
    imu_data.angular_velocity.x = airsim_img_data.angular_velocity(0);
    imu_data.angular_velocity.y = airsim_img_data.angular_velocity(1);
    imu_data.angular_velocity.z = airsim_img_data.angular_velocity(2);
    imu_data.linear_acceleration.x = airsim_img_data.linear_acceleration(0);
    imu_data.linear_acceleration.y = airsim_img_data.linear_acceleration(1);
    imu_data.linear_acceleration.z = airsim_img_data.linear_acceleration(2);
    pub_imu.publish(imu_data);
}

void AirsimNode::getMagneticData(msr::airlib::MultirotorRpcLibClient* client) {
    msr::airlib::MagnetometerData airsim_magnetic_data;
    airsim_magnetic_data = client->getMagnetometerdata(1.0 / DATA_FREQ);
    magnetic_data.header.frame_id = "base_link";
    magnetic_data.header.stamp = ros::Time::now();
    magnetic_data.magnetic_field.x =
        airsim_magnetic_data.magnetic_field_body(0);
    magnetic_data.magnetic_field.y =
        airsim_magnetic_data.magnetic_field_body(1);
    magnetic_data.magnetic_field.z =
        airsim_magnetic_data.magnetic_field_body(2);
    pub_magnetic.publish(magnetic_data);
}
void AirsimNode::getGPSData(msr::airlib::MultirotorRpcLibClient* client) {
    msr::airlib::GeoPoint airsim_aps_data;
    airsim_aps_data = client->getGpsLocation();
    gps_data.header.frame_id = "base_link";
    gps_data.header.stamp = ros::Time::now();
    gps_data.latitude = airsim_aps_data.latitude;
    gps_data.longitude = airsim_aps_data.longitude;
    gps_data.altitude = airsim_aps_data.altitude;
    pub_gps.publish(gps_data);
}
void AirsimNode::getBarometerData(msr::airlib::MultirotorRpcLibClient* client) {
    msr::airlib::BarometerData airsim_barometer_data;
    airsim_barometer_data = client->getBarometerdata(1.0 / DATA_FREQ);
    barometer_data.header.frame_id = "base_link";
    barometer_data.header.stamp = ros::Time::now();
    barometer_data.vector.x = airsim_barometer_data.altitude;  // altitude meter
    barometer_data.vector.y = airsim_barometer_data.pressure;  // pressure
                                                               // Pascal
    pub_barometer.publish(barometer_data);
}

bool AirsimNode::takeoff() {
    cout<<"AAAA   "<<control_client->isApiControlEnabled()<<endl;
    control_client->enableApiControl(true);
        cout<<"AAAA   "<<control_client->isApiControlEnabled()<<endl;
    control_client->armDisarm(true);
    // std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
    float takeoffTimeout = 2;
    control_client->takeoff(takeoffTimeout);
}
bool AirsimNode::land() {
    float takeoffTimeout = 5;
    control_client->land(takeoffTimeout);
    control_client->armDisarm(false);
}
bool AirsimNode::move(float pitch, float roll, float throttle, float yaw,
                      float duration) {
    ;
}
bool AirsimNode::hover() { ; }

void AirsimNode::connect() {
    control_client->confirmConnection();
}


void AirsimNode::flightDataThread(){
    msr::airlib::MultirotorRpcLibClient* client = new msr::airlib::MultirotorRpcLibClient(ip_adress);
    client->confirmConnection();
    ros::Rate rate(80);
    while(RUNNING_FLAG){
        getAirsimData(client);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Get flightData Thread Exit");
}

void AirsimNode::imageFrontRgbThread(){
    msr::airlib::MultirotorRpcLibClient* client = new msr::airlib::MultirotorRpcLibClient(ip_adress);
    client->confirmConnection();
    ros::Rate rate(20);
    while(RUNNING_FLAG){
        getImageFrontRgbData(client);
        ros::spinOnce();
        rate.sleep();
    }
        ROS_INFO("Get imageFrontRgb Thread Exit");
}

void AirsimNode::imageFrontDepthThread(){
    msr::airlib::MultirotorRpcLibClient* client = new msr::airlib::MultirotorRpcLibClient(ip_adress);
    client->confirmConnection();
    ros::Rate rate(20);
    while(RUNNING_FLAG){
        getImageFrontDepthData(client);
        ros::spinOnce();
        rate.sleep();
    }
     ROS_INFO("Get imageFrontDepth Thread Exit");
}

void AirsimNode::imageDownRgbThread(){
    msr::airlib::MultirotorRpcLibClient* client = new msr::airlib::MultirotorRpcLibClient(ip_adress);
    client->confirmConnection();
    ros::Rate rate(20);
    while(RUNNING_FLAG){
        getImageDownRgbData(client);
        ros::spinOnce();
        rate.sleep();
    }
     ROS_INFO("Get imageDownRgb Thread Exit");
}

void AirsimNode::run(){
    RUNNING_FLAG=1;
    std::thread t1(&AirsimNode::imageDownRgbThread,this);
    std::thread t2(&AirsimNode::imageFrontRgbThread,this);
    std::thread t3(&AirsimNode::imageFrontDepthThread,this);
    std::thread t4(&AirsimNode::flightDataThread,this);
    
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    ROS_INFO("Airsim Run Exit");
}

