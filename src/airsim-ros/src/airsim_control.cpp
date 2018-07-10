#include "airsim_control.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;

AirsimControl::AirsimControl()
{
    sub_imu = nh.subscribe("airsim/imu", 1, &AirsimControl::cb_imu, this);
    sub_gps = nh.subscribe("airsim/gps", 1, &AirsimControl::cb_gps, this);
    sub_magnetic = nh.subscribe("airsim/magnetic", 1, &AirsimControl::cb_magnetic, this);
    sub_baromrter = nh.subscribe("airsim/barometer", 1, &AirsimControl::cb_barometer, this);
    sub_object_front = nh.subscribe("airsim/object/front", 1, &AirsimControl::cb_object_front, this);
    sub_object_down = nh.subscribe("airsim/object/down", 1, &AirsimControl::cb_object_down, this);
    detect_num = -1;
    memset(target_mode, 0, sizeof(int) * 11);

    control_client = new msr::airlib::MultirotorRpcLibClient("192.168.1.100");
    control_client->confirmConnection();
}

void AirsimControl::cb_imu(const sensor_msgs::Imu &msg)
{
    msg_imu = msg;
}
void AirsimControl::cb_gps(const sensor_msgs::NavSatFix &msg)
{
    msg_gps = msg;
}
void AirsimControl::cb_magnetic(const sensor_msgs::MagneticField &msg)
{
    msg_magnetic = msg;
}
void AirsimControl::cb_barometer(const geometry_msgs::Vector3Stamped &msg)
{
    msg_barometer = msg;
}
void AirsimControl::cb_object_front(const gf_perception::ObjectList &msg)
{
    ;
}
void AirsimControl::cb_object_down(const gf_perception::ObjectList &msg)
{
    msg_objects_down = msg;
    object_down = gf_perception::Object();
    if (detect_num == -1)
        detect_num = 1;
    if (msg.count > 0)
    {
        for (int i = 0; i < msg.object.size(); i++)
        {
            target_mode[msg.object[i].number] = 2;
            if (msg.object[i].number == detect_num)
                object_down = msg.object[i];
            
                
        }
    }
}

void AirsimControl::run()
{
    int detect_state = 0; //0-寻找,1-下视RGB检测 2-RGB图像检测,3-RGB+深度图像检测,4-圆圈检测，5-穿越/降落
    //0-未知,1-板子.2-停机坪
    target_mode[0] = 2;
    target_mode[1] = 2;
    target_mode[2] = 2;
    target_mode[10] = 2;

    //PID
    double control_throttle;
    double control_pitch;
    double control_roll;
    //高度
    PIDctrl pid_height;
    pid_height.init(0.08, 0.0003, 1.5, 1000000);
    double target_height = 15;
    PIDctrl pid_pitch;
    pid_pitch.init(0.00005, 0, 0.01 , 0.21);
    PIDctrl pid_roll;
    pid_roll.init(0.00005, 0, 0.01 , 0.21);

    //标志位
    int count_detect_down_cam = 0;
    int lost_detect_down_cam = 0;
    int count_center_down_cam=0;

    ros::Rate rate(30);

    while (1)
    {
        sleep(1);
        ros::spinOnce();
        if (msg_barometer.vector.x != 0)
            break;
    }
    takeoff();
    target_height = msg_barometer.vector.x;
    while (ros::ok())
    {
        ROS_INFO("Detect Num: %d  Mode: %d State: %d", detect_num, target_mode[detect_num], detect_state);
        control_roll = 0;
        control_pitch = 0;
        //数字
        if (detect_num > 0 && detect_num < 11)
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
                //寻找目标
                if (detect_state == 0)
                {
                    ROS_INFO("Finding  target height %f", target_height);
                    if (object_down.number == detect_num)
                    {
                        count_detect_down_cam++;
                    }
                    else
                    {
                        count_detect_down_cam -= 1;
                        if (count_detect_down_cam < 0)
                            count_detect_down_cam = 0;
                    }
                    if (count_detect_down_cam > 3)
                    {
                        detect_state = 1;
                        count_detect_down_cam=0;
                    }
                    else
                    {

                        if (target_height < 40)
                            target_height = target_height + 0.05;
                    }
                }
                //下视检测
                else if (detect_state == 1)
                {
                    ROS_INFO("Down Cam %d targetnum %d(%f,%f)",object_down.number, detect_num, object_down.center.x, object_down.center.y);
                    if (object_down.number == detect_num)
                    {
                        lost_detect_down_cam=0;
                        control_roll = pid_roll.calc(object_down.center.x - 320.0);
                        control_pitch = pid_pitch.calc(object_down.center.y * 1.0 - 240.0);
                        ROS_ERROR("R %f,%f",object_down.center.x - 320.0,control_roll);
                        ROS_ERROR("P %f,%f",object_down.center.y - 240.0,control_pitch);
                        double dx=abs(object_down.center.x - 320);
                        double dy=abs(object_down.center.y - 240);

                        if (dx < 10)
                            control_roll = 0;
                        if (dy < 10)
                            control_pitch = 0;
                            ROS_ERROR("%f",object_down.size.x * object_down.size.y);
                        if ((object_down.size.x * object_down.size.y) < 30000 && dy<100 && dx<150)
                        {
                            target_height = target_height - 0.05;
                            
                        }
                        if ( dy<20 && dx<20 && (object_down.size.x * object_down.size.y) > 30000){
                            count_center_down_cam++;
                        }
                        else{
                            count_center_down_cam=0;
                        }
                        if (count_center_down_cam>4)   {
                            detect_state=5;
                            count_center_down_cam=0;
                        }

                        
                    }
                    else
                    {
                        lost_detect_down_cam++;
                        if (lost_detect_down_cam > 10)
                        {
                            detect_state = 0;
                        }
                    }
                }
                else if (detect_state==5){
                    ROS_INFO("LAND");
                    land();
                    ROS_INFO("LANd Finish");
                    detect_num++;
                    detect_state = 0;
                    takeoff();
                }
            }
        }
        //二维码
        else
        {
            ;
        }
        
        double d_height = target_height - msg_barometer.vector.x;
        control_throttle = pid_height.calc(d_height);
        ROS_INFO("Target :%f H: %f  D: %f", target_height, d_height, control_throttle);
        ROS_INFO("P: %f  R: %f", control_pitch, control_roll);
        move(control_pitch, control_roll, control_throttle, 0, 5);
        ros::spinOnce();
        rate.sleep();
    }
}

bool AirsimControl::takeoff()
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->armDisarm(true);
    ROS_INFO("Command : Take Off");
    if (msg_barometer.vector.x == 0)
    {
        ROS_ERROR("Cannot get barometer data!");
        return 0;
    }

    initial_height = msg_barometer.vector.x;
    PIDctrl pid_height;
    pid_height.init(0.12, 0.0018, 1.5, 5);
    int count = 0;
    while (1)
    {
        double d_height = initial_height + 5 - msg_barometer.vector.x;
        double d_throttle = pid_height.calc(d_height);
        move(0, 0, d_throttle, 0, 5);
        if (abs(d_height) < 0.1)
            break;
        // count++;
        else
            count = 0;
        cout << d_height << "   " << count << endl;
        usleep(10000);
        ros::spinOnce();
    }
    control_client->hover();
    ROS_INFO("Take Off Finish");
}

// bool AirsimControl::land()
// {
//     PIDctrl pid_height;
//     pid_height.init(0.06, 0.0003, 1.5, 5);
//     double land_target_altitude = msg_barometer.vector.x;
//     int highest_altitude = 25;
//     int get_to_highest_altitude_flag = 0, start_land_flag = 0;

//     double last_altitude = 0;
//     int land_complete_cnt = 0;
//     control_client->hover();
//     sleep(1);
//     while (1)
//     {
//         //上升中：
//         // if (get_to_highest_altitude_flag == 0)
//         // {
//         //     land_target_altitude = highest_altitude;
//         // }
//         // if (get_to_highest_altitude_flag == 0 && msg_barometer.vector.x > highest_altitude - 0.5)
//         // {
//         //     get_to_highest_altitude_flag = 1;
//         //     start_land_flag = 1;
//         // }
//         //下降中：
//         // if (start_land_flag == 1)
//         // {
//             land_target_altitude -= 0.03;
//         // }
//         //降落完成：
//         land_complete_cnt++;
//         if (land_complete_cnt >= 300)
//         {
//             //如果3秒前后高度差大于1,则判断仍在下降
//             if (abs(msg_barometer.vector.x - last_altitude) > 0.8)
//             {
//                 land_complete_cnt = 0;
//                 last_altitude = msg_barometer.vector.x;
//             }
//             //否则降落成功：跳出循环
//             else
//             {
//                 break;
//             }
//         }
//         //PID部分
//         double d_height = land_target_altitude - msg_barometer.vector.x;
//         double d_throttle = pid_height.calc(d_height);
//         move(0, 0, d_throttle, 0, 5);

//         cout << "   target_altitude:" << land_target_altitude << "real_altitude :" << msg_barometer.vector.x << endl;
//         usleep(10000);
//         ros::spinOnce();
//     }
//     ROS_INFO("Land complete ");
// }

bool AirsimControl::land()
{
    control_client->hover();
    sleep(1);
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    float takeoffTimeout = 100;
    control_client->land(takeoffTimeout);
    sleep(1);
    control_client->armDisarm(false);
    ROS_INFO("Command : Land");

}

bool AirsimControl::move(float pitch, float roll, float throttle, float yaw,
                         float duration)
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->moveByAngleThrottle(pitch, roll, throttle, yaw, duration); //ROS_INFO("Command : Move Pitch-%f Roll-%f Throtele-%f Yaw-%f Dur%f",pitch, roll, throttle, yaw,duration);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AirsimControl airsim_ctrl;
    airsim_ctrl.run();
}