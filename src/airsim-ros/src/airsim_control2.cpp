#include "airsim_control.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
using namespace std;
double kp, ki, kd, pid_max, kp2, ki2, kd2, pid_max2;
AirsimControl::AirsimControl()
{
    sub_imu = nh.subscribe("airsim/imu/fusion", 1, &AirsimControl::cb_imu, this);
    sub_gps = nh.subscribe("airsim/gps", 1, &AirsimControl::cb_gps, this);
    sub_magnetic = nh.subscribe("airsim/magnetic", 1, &AirsimControl::cb_magnetic, this);
    sub_baromrter = nh.subscribe("airsim/barometer", 1, &AirsimControl::cb_barometer, this);
    sub_object_front = nh.subscribe("gaofen_detection/number", 1, &AirsimControl::cb_object_front, this);
    sub_object_down = nh.subscribe("airsim/object/down", 1, &AirsimControl::cb_object_down, this);
    sub_object_circle = nh.subscribe("airsim/depth/circle", 1, &AirsimControl::cb_object_circle, this);
    sub_depth_count = nh.subscribe("airsim/depth/count", 1, &AirsimControl::cb_depth_count, this);
    pub_front_camera_pose = nh.advertise<std_msgs::Int16>("airsim/front_camera/pose_state", 1);
    pub_height_filter = nh.advertise<geometry_msgs::Vector3Stamped>("airsim/height_filter", 1);
    detect_num = -1;
    memset(target_mode, 0, sizeof(int) * 11);
    memset(target_mode_count, 0, sizeof(int) * 11 * 2);

    control_client = new msr::airlib::MultirotorRpcLibClient("192.168.1.100");
    control_client->confirmConnection();
    error_code = 0;

    running_count = 0;
    leftright_count = -1;
    forward_count = 0;
    search_flag = 0;
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
void AirsimControl::cb_depth_count(const geometry_msgs::Vector3Stamped &msg)
{
    msg_pre_depth_count = msg_depth_count;
    msg_depth_count = msg;
}
void AirsimControl::cb_object_front(const gf_perception::ObjectList &msg)
{
    msg_objects_front = msg;
    object_front = gf_perception::Object();
    if (msg.count > 0)
    {
        for (int i = 0; i < msg.object.size(); i++)
        {
            ROS_ERROR("****FIND OBJ %d %d %d %d", msg.object[i].number, msg.object[i].type, target_mode_count[msg.object[i].number][0], target_mode_count[msg.object[i].number][1]);
            ROS_ERROR("SIZE %f", msg.object[i].size.x * msg.object[i].size.y);
            if (msg.object[i].type == 2 && msg.object[i].center.y > 450 && msg.object[i].size.x * msg.object[i].size.y > 5000)
                continue;

            if (msg.object[i].number == detect_num)
                object_front = msg.object[i];
            if (msg.object[i].number >= detect_num && msg.object[i].number < detect_num + 3)
            {
                if (msg.object[i].type == 1)
                {
                    target_mode_count[msg.object[i].number][0] += 1;
                }
                else if (msg.object[i].type == 2)
                {
                    target_mode_count[msg.object[i].number][1] += 2;
                }
                else if (msg.object[i].type == 3)
                {
                    target_mode_count[msg.object[i].number][1] += 5;
                }
            }

            if (target_mode_count[msg.object[i].number][0] > target_mode_count[msg.object[i].number][1] * 4 && target_mode_count[msg.object[i].number][0] > 30)
            {
                target_mode[msg.object[i].number] = 1;
            }
            if (target_mode_count[msg.object[i].number][1] > target_mode_count[msg.object[i].number][0] * 2 && target_mode_count[msg.object[i].number][1] > 30)
            {
                target_mode[msg.object[i].number] = 2;
            }
        }
    }
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
            // target_mode[msg.object[i].number] = 2;
            if (msg.object[i].number == detect_num)
            {
                object_down = msg.object[i];
                target_mode[msg.object[i].number] = 2;
            }
        }
    }
}
void AirsimControl::cb_object_circle(const geometry_msgs::Vector3Stamped &msg)
{
    msg_circle = msg;
}

//均值滤波 -> 高度
////////////////////////////////////////////
#define filter_N 5

float filter_seq_height[filter_N] = {0};
float filtered_height = 0;

float sum(float a[], size_t len)
{
    float sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += a[i];
    }
    return sum;
}

float find_min(float a[], size_t len)
{
    float min = a[0];

    for (int j = 1; j < len; j++)
    {
        if (a[j] < min)
            min = a[j];
    }

    return min;
}

float find_max(float a[], size_t len)
{
    float max = a[0];

    for (int j = 1; j < len; j++)
    {
        if (a[j] > max)
            max = a[j];
    }
    return max;
}
////////////////////////////////////////////

//一阶滞后滤波
////////////////////////////////////////////
#define one_a 0.8

float last_filter_value = 0;
float this_time_value = 0;
float one_filter_back(float new_value, float last_filter_value)
{
    return one_a * last_filter_value + (1 - one_a) * new_value;
}

////////////////////////////////////////////

void AirsimControl::run()
{
    int detect_state = 0; //0-寻找,1-下视RGB检测 2-RGB图像检测,3-RGB+深度图像检测,4-圆圈检测，5-穿越/降落
    //0-未知,1-板子.2-停机坪
    target_mode[0] = 2;
    target_mode[1] = 0;
    target_mode[2] = 0;
    target_mode[3] = 0;
    target_mode[4] = 0;
    target_mode[5] = 0;
    target_mode[10] = 2;

    //PID
    double control_throttle;
    double control_pitch;
    double control_roll;
    double control_yaw;
    //高度
    PIDctrl pid_height;
    //起飞PID
    pid_height.init(kp, ki, kd, pid_max); //0.12, 0.0018, 1.5, 5
    double target_height = 15;
    PIDctrl pid_pitch;
    pid_pitch.init(0.0002, 0, 0.006, 0.025);
    PIDctrl pid_roll;
    pid_roll.init(0.0002, 0, 0.006, 0.025);
    PIDctrl pid_yaw;
    pid_yaw.init(1, 0, 0, 1);
    double target_yaw;

    //标志位
    int hover_flag = 0;
    //未知
    int rotate_flag = 1; //0-stay 1 center-left  2-left-center 3 center-right 4 right-center
    int lost_count = 0;
    int circle_count = 0;
    int circle_center_count = 0;
    int go_through_count = 0;
    int up_down_mode = 0;
    //下视
    int count_detect_down_cam = 0;
    int lost_detect_down_cam = 0;
    int count_center_down_cam = 0;
    int down_search_mode = 0; //0 - 前视 1 -向右 2 - 向左
    float target_distance, pre_target_distance;
    ros::Rate rate(30);

    while (1)
    {
        sleep(1);
        ros::spinOnce();
        if (msg_barometer.vector.x != 0)
            break;
    }
    initial_height = msg_barometer.vector.x;
    takeoff(&pid_height);

    //起飞切换到move , 更换高度PID参数:
    //float dynamic_ki2 = 0.00108;
    pid_height.init(kp2, ki2, kd2, pid_max2); //0.08, 0.0003, 1.5, 5

    // //正式飞行前瞎飞计时:
    // int xiafei_cnt = 0 , stop_cnt_flag = 0;

    target_height = msg_barometer.vector.x;
    initial_yaw = tf::getYaw(msg_imu.orientation);
    target_yaw = initial_yaw;

    ros::NodeHandle private_nh("~");
    while (ros::ok())
    {

        /////
        //修改PID参数:
        private_nh.getParam("kp", kp);
        private_nh.getParam("ki", ki);
        private_nh.getParam("kd", kd);
        private_nh.getParam("max", pid_max);
        private_nh.getParam("kp2", kp2);
        private_nh.getParam("ki2", ki2);
        private_nh.getParam("kd2", kd2);
        private_nh.getParam("max2", pid_max2);
        pid_height.init(kp2, ki2, kd2, pid_max2);
        ///////////////////////////////////////////

        // ROS_ERROR("Detect Num: %d  Mode: %d State: %d", detect_num, target_mode[detect_num], detect_state);
        control_roll = 0;
        control_pitch = 0;
        control_yaw = 0;
        hover_flag = 0;
        std_msgs::Int16 front_cam_pose_msg;
        //数字
        if (detect_num > 0 && detect_num < 11)
        {
            //板子未知类型 探索
            if (target_mode[detect_num] == 0)
            {
                //寻找
                ROS_ERROR("Detect : %d  Target Unknown & Search  %d,%d", detect_num, up_down_mode,down_search_mode);

                if (up_down_mode == 2 || up_down_mode == 3)
                {
                    if (msg_depth_count.vector.z > 0.6)
                    {
                        target_height += 0.05;
                    }
                    // if (msg_depth_count.vector.z ==0){
                    //     target_height -= 0.05;
                    // }
                    // else if (msg_depth_count.vector.z > 0.7 && msg_depth_count.vector.z <0.8)
                    //     target_height += 0.01;
                    // else if (msg_depth_count.vector.z < 0.4  && msg_depth_count.vector.z > 0)
                    //     target_height -= 0.01;
                }
                else if (up_down_mode == 0 || up_down_mode == 1)
                    target_height = initial_height + 7;
                if (down_search_mode == 0)
                {
                    down_search_mode = 1;
                    leftright_count = leftright_count / abs(leftright_count);
                }

                if (down_search_mode == 1)
                {
                    ROS_INFO("%d ERRRR  %f,%f,%f", leftright_count, msg_depth_count.vector.x, msg_depth_count.vector.y, msg_depth_count.vector.z);
                    search(control_pitch, control_roll, target_yaw);
                    if (abs(leftright_count) == 1)
                    {
                        if (up_down_mode == 3)
                        {
                            down_search_mode = 2;
                            up_down_mode = -1;
                            target_height = initial_height + 7;
                        }
                        if (up_down_mode == 1)
                        {
                            target_height = initial_height + 3;
                            down_search_mode = 2;
                        }
                        up_down_mode++;
                    }
                }

                if (down_search_mode == 2)
                {
                    if (abs(target_height - msg_barometer.vector.x) < 0.3)
                    {
                        if (up_down_mode == 0)
                        {
                            if (go_forward(control_pitch, control_roll, target_yaw))
                            {
                                down_search_mode = 1;
                            }
                        }
                        else{
                            down_search_mode = 1;
                        }
                    }
                }
            }
            //障碍圈
            if (target_mode[detect_num] == 1)
            {
                pid_yaw.init(0.15, 0, 0, 0.1);
                if (detect_state == 0)
                {

                    if (object_front.number != detect_num)
                    {
                        lost_count++;
                        double d = 0.8;
                        if (rotate_flag % 2 == 0)
                        {
                            target_yaw = initial_yaw - d;
                        }
                        else if (rotate_flag % 2 == 1)
                        {
                            target_yaw = initial_yaw + d;
                        }
                        if (abs(tf::getYaw(msg_imu.orientation) - target_yaw) < 0.1)
                            rotate_flag++;
                        if (up_down_mode == 0)
                        {
                            target_height += 0.05;
                            if (target_height > initial_height + 6)
                                up_down_mode = 1;
                        }
                        else if (up_down_mode == 1)
                        {
                            target_height -= 0.05;
                            if (target_height < initial_height)
                                up_down_mode = 0;
                        }
                        if (lost_count % 20 == 0)
                        {
                            if (detect_num == 4 && target_height < initial_height + 6)
                            {
                                // target_height += 0.03;
                                control_pitch = 0.015;
                            }
                            else if (detect_num == 5 && target_height > initial_height - 1)
                            {
                                // target_height -= 0.03;
                                control_pitch = 0.015;
                            }
                        }
                    }
                    else
                    {
                        target_yaw = initial_yaw;
                        double theta = (object_front.center.x - 320) / 320.0 * 50.0 / 180.0 * 3.14;
                        detect_state = 2;
                    }
                }
                else if (detect_state == 2)
                {

                    if (object_front.number != detect_num)
                    {
                        lost_count++;
                        if (lost_count > 3)
                        {
                            if (detect_num == 4 && target_height < initial_height + 6)
                            {
                                target_height += 0.03;
                            }
                            else if (detect_num == 5 && target_height > initial_height - 1)
                            {
                                target_height -= 0.03;
                            }
                            detect_state = 0;
                            lost_count = 0;
                            control_client->hover();
                            sleep(1);
                        }
                    }
                    else
                    {
                        lost_count = 0;
                        target_yaw = initial_yaw;
                        double theta = (object_front.center.x - 320) / 320.0 * 50.0 / 180.0 * 3.14;
                        double speed = 0.01;
                        if (abs(target_yaw - tf::getYaw(msg_imu.orientation) > 0.2))
                        {
                            if (object_front.size.x * object_front.size.y < 200)
                                control_pitch = -0.002;
                            else if (object_front.size.x * object_front.size.y > 200)
                                control_pitch = 0;
                            else if (object_front.size.x * object_front.size.y > 300)
                                control_pitch = 0.001;
                            control_roll = pid_roll.calc((object_front.center.x - 320.0) * 1.5);
                            if (object_front.center.y < 240)
                                target_height += 0.008;
                            if (object_front.center.y > 240)
                                target_height -= 0.008;
                        }
                        else
                        {
                            if (object_front.size.x * object_front.size.y > 900)
                                hover_flag = 1;
                            else if (object_front.size.x * object_front.size.y < 200)
                                control_pitch = -0.0025;
                            else if (object_front.size.x * object_front.size.y > 200)
                                control_pitch = -0.0015;
                            control_roll = pid_roll.calc(object_front.center.x - 320.0);
                            if (object_front.center.y < 240)
                                target_height += 0.002;
                            if (object_front.center.y > 240)
                                target_height -= 0.002;
                        }
                        if (abs(target_yaw - tf::getYaw(msg_imu.orientation) < 0.05))
                        {
                            if (msg_circle.vector.x && msg_circle.vector.y)
                            {
                                circle_count++;
                                if (circle_count > 3)
                                {
                                    circle_count = 0;
                                    detect_state = 3;
                                    control_client->hover();
                                    sleep(1);
                                }
                            }
                            else
                            {
                                circle_count = 0;
                            }
                        }
                        ROS_INFO("%f  thet %f  SIZe %f", (object_front.center.x - 320), theta, object_front.size.x * object_front.size.y);
                    }
                }
                else if (detect_state == 3)
                {
                    target_yaw = initial_yaw;
                    if (msg_circle.vector.x && msg_circle.vector.y)
                    {
                        if (msg_circle.vector.y > 245)
                            target_height -= 0.02;
                        else if (msg_circle.vector.y < 225)
                        {
                            target_height += 0.01;
                        }
                        control_roll = pid_roll.calc(msg_circle.vector.x - 320);

                        if (abs(msg_circle.vector.y - 240) < 30 && abs(msg_circle.vector.x - 320) < 30)
                        {

                            if (msg_circle.vector.z > 70)
                                control_pitch = 0.01;
                            if (msg_circle.vector.z > 70)
                                control_pitch = 0.003;
                            else if (msg_circle.vector.z > 60)
                                control_pitch = 0.001;
                            else
                                control_pitch = -0.001;
                        }

                        if (abs(msg_circle.vector.y - 240) < 15 && abs(msg_circle.vector.x - 320) < 15 && msg_circle.vector.z > 80)
                        {
                            hover_flag = 1;
                            circle_center_count++;
                            if (circle_center_count > 4)
                            {
                                detect_state = 4;
                                circle_center_count = 0;
                                control_client->hover();
                                sleep(1);
                            }
                        }
                        else
                        {
                            circle_center_count = 0;
                        }
                    }
                    else
                    {
                        hover_flag = 1;
                    }
                }
                else if (detect_state == 4)
                {
                    if (msg_circle.vector.y > 240)
                        target_height -= 0.02;
                    else
                    {
                        target_height += 0.02;
                    }
                    // target_height+=0.05;
                    go_through_count++;
                    if (go_through_count < 50)
                        control_pitch = -0.05;
                    else if (go_through_count < 100)
                        control_pitch = 0.05;
                    else
                    {
                        control_client->hover();
                        sleep(1);
                        go_through_count = 0;
                        detect_state = 0;
                        detect_num++;
                    }
                    // if (go_through_count > 30)
                    // {
                    //     for(int i=0;i<5;i++){
                    //         move(0.2, 0, control_throttle, 0, 5);
                    //     }
                    //     control_client->hover();
                    //     sleep(1);
                    //     detect_state=0;
                    //     detect_num++;
                    // }
                }
                else
                {
                    detect_state = 0;
                    lost_count = 0;
                    rotate_flag = 0;
                }
            }
            //停机坪
            if (target_mode[detect_num] == 2)
            {
                pid_yaw.init(1, 0, 0, 1);
                pre_target_distance = target_distance;
                target_distance = sqrt(pow(object_down.center.y - 240, 2) + pow(object_down.center.x - 320, 2));
                //寻找目标
                if (detect_state == 0)
                {
                    ROS_ERROR("Detect : %d  Target Down  & Search", detect_num);

                    if (target_height < 17)
                        target_height += 0.02;

                    if (object_down.number == detect_num)
                    {
                        count_detect_down_cam++;
                        lost_detect_down_cam = 0;
                        ROS_INFO("DIS  %f  %f ", target_distance, pre_target_distance);
                        if (count_detect_down_cam > 5 && down_search_mode != 2)
                        {
                            if (target_distance > pre_target_distance || target_distance < 150)
                            {
                                down_search_mode = 2;
                                go_through_count = 0;
                            }
                        }
                    }
                    else
                    {
                        count_detect_down_cam -= 1;
                        if (count_detect_down_cam < 0)
                            count_detect_down_cam = 0;
                        lost_detect_down_cam++;
                    }

                    ROS_INFO("Finding  target  height %d  %d %d", count_detect_down_cam, lost_detect_down_cam, down_search_mode);

                    if (down_search_mode == 0) //向前一段
                    {
                        if (go_forward(control_pitch, control_roll, target_yaw))
                        {
                            down_search_mode = 1;
                            if (detect_num == 1 || detect_num == 10) //1号或10号 往右飞
                                leftright_count = 1;
                            else
                                leftright_count = -1;
                        }
                    }
                    if (down_search_mode == 1) //左右循环
                    {
                        search(control_pitch, control_roll, target_yaw);
                    }

                    if (down_search_mode == 2) //刹车
                    {
                        go_through_count++;
                        if (go_through_count < 10)
                        {
                            control_roll = (leftright_count > 0) ? -0.15 : 0.15;
                            ROS_ERROR("FFFFF  %d", go_through_count);
                        }
                        else if (go_through_count == 10)
                        {
                            control_client->hover();
                            sleep(2);
                        }
                        else if (go_through_count > 20)
                        {
                            target_yaw = initial_yaw;
                            ROS_ERROR("FFFFF  %d", go_through_count);
                            if (count_detect_down_cam > 5 && abs(tf::getYaw(msg_imu.orientation) - target_yaw) < 0.02)
                            {
                                detect_state = 1;
                                count_detect_down_cam = 0;
                                down_search_mode = 0;
                                go_through_count = 0;
                                leftright_count = leftright_count / abs(leftright_count);
                                up_down_mode = 0;
                            }
                            if (lost_detect_down_cam > 10)
                            {
                                target_height += 0.01;
                                if (target_height > 19)
                                {
                                    down_search_mode = 1;
                                }
                            }
                        }
                    }
                }
                //下视检测
                else if (detect_state == 1)
                {
                    ROS_ERROR("Detect : %d  Target Down  & Detect  ( %f,%f)", detect_num, object_down.center.x, object_down.center.y);
                    if (object_down.number == detect_num)
                    {
                        lost_detect_down_cam = 0;
                        double drx = (object_down.center.x - 320.0) * (msg_barometer.vector.x - initial_height) / 268.3 * 100;
                        double dry = (object_down.center.y - 240.0) * (msg_barometer.vector.x - initial_height) / 268.3 * 100;
                        control_roll = pid_roll.calc(drx);
                        control_pitch = pid_pitch.calc(dry);
                        ROS_INFO("R %f, P%f, Size %f", object_down.center.x - 320.0, object_down.center.y - 240.0, object_down.size.x * object_down.size.y);
                        ROS_INFO("DX  %f  DY %f", drx, dry);

                        double dx = abs(object_down.center.x - 320);
                        double dy = abs(object_down.center.y - 240);

                        if (dx < 25 && dy < 25)
                            hover_flag = 1;

                        // if (dy < 70 && dx < 80 && target_height > 16)
                        // {
                        //     target_height = target_height - 0.015;
                        // }
                        if (dy < 25 && dx < 25)
                        {
                            count_center_down_cam++;
                        }
                        else
                        {
                            count_center_down_cam = 0;
                        }
                        if (count_center_down_cam > 3)
                        {
                            detect_state = 5;
                            count_center_down_cam = 0;
                        }
                    }
                    else
                    {
                        target_height = target_height + 0.02;
                        if (target_height > 21)
                        {
                            target_height = 21;
                            control_pitch = -0.005;
                        }
                        lost_detect_down_cam++;
                        if (lost_detect_down_cam > 100)
                        {
                            detect_state = 0;
                        }
                    }
                }
                else if (detect_state == 5)
                {

                    ROS_INFO("LAND");
                    land();
                    ROS_INFO("LANd Finish");
                    detect_num++;
                    detect_state = 0;
                    pid_height.init(kp, ki, kd, pid_max);
                    takeoff(&pid_height);
                }
                else
                {
                    detect_state = 0;
                }

                // if (tf::getYaw(msg_imu.orientation) < initial_yaw + 0.8 && tf::getYaw(msg_imu.orientation) > initial_yaw - 0.8)
                // {
                //     front_cam_pose_msg.data = 0;
                // }
                // else if (tf::getYaw(msg_imu.orientation) > initial_yaw + 0.8)
                // {
                //     front_cam_pose_msg.data = 2;
                // }
                // else
                // {
                //     front_cam_pose_msg.data = 1;
                // }
            }
        }
        //二维码
        else
        {
            ;
        }

        float current_yaw = tf::getYaw(msg_imu.orientation);
        if (target_yaw > 0 && current_yaw < 0 && (target_yaw - current_yaw) > 3.14)
        {
            control_yaw = pid_yaw.calc(current_yaw + 6.28 - target_yaw);
        }
        else if (target_yaw < 0 && current_yaw > 0 && (current_yaw - target_yaw) > 3.14)
        {
            control_yaw = pid_yaw.calc(-(target_yaw + 6.28 - current_yaw));
        }
        else
        {
            control_yaw = pid_yaw.calc(current_yaw - target_yaw);
        }

        //filter:
        // for (int i = 0; i < filter_N - 1; i++)
        // {
        //     filter_seq_height[i] = filter_seq_height[i + 1];
        // }

        // filter_seq_height[filter_N - 1] = msg_barometer.vector.x;

        // filtered_height = (sum(filter_seq_height, filter_N) - find_max(filter_seq_height, filter_N) - find_min(filter_seq_height, filter_N)) / (filter_N - 2);

        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);
        geometry_msgs::Vector3Stamped msg_h;
        msg_h.header.stamp = ros::Time::now();
        msg_h.vector.x = filtered_height;
        msg_h.vector.y = current_yaw;
        msg_h.vector.z = target_height;
        pub_height_filter.publish(msg_h);

        ////////////////////////////

        double d_height = target_height - filtered_height; //msg_barometer.vector.x;
        control_throttle = pid_height.calc(d_height);
        ROS_INFO("Target H:%f dH: %f Target Y:%f  dY,%f", target_height, d_height, target_yaw, tf::getYaw(msg_imu.orientation) - target_yaw);
        ROS_INFO("P: %f  R: %f  Y: %f  T: %f", control_pitch, control_roll, control_yaw, control_throttle);
        if (hover_flag)
        {
            control_client->hover();
            usleep(300000);
        }
        else
            move(control_pitch, control_roll, control_throttle + 0.58, control_yaw, 5);
        //pub_front_camera_pose.publish(front_cam_pose_msg);
        ros::spinOnce();
        rate.sleep();
    }
}

bool AirsimControl::takeoff(PIDctrl *pid)
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

    // PIDctrl pid_height;
    // pid_height.init(0.12, 0.0018, 1.5, 5); // 0.12, 0.0018, 1.5, 5

    int count = 0;
    while (1)
    {
        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);

        double d_height;

        d_height = initial_height + 8 - filtered_height;

        double d_throttle = pid->calc(d_height);
        move(0, 0, d_throttle, 0, 5);
        if (abs(d_height) < 1)
            break;
        // count++;
        else
            count = 0;
        usleep(33000);
        ros::spinOnce();
    }
    control_client->hover();
    ROS_INFO("Take Off Finish");
}

bool AirsimControl::land()
{
    PIDctrl pid_height;
    pid_height.init(0.06, 0.0003, 1.5, 5);
    double land_target_altitude = msg_barometer.vector.x;
    int highest_altitude = 25;
    int get_to_highest_altitude_flag = 0, start_land_flag = 0;

    double last_altitude = 0;
    int land_complete_cnt = 0;
    control_client->hover();
    sleep(1);
    while (1)
    {
        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);

        //上升中：
        // if (get_to_highest_altitude_flag == 0)
        // {
        //     land_target_altitude = highest_altitude;
        // }
        // if (get_to_highest_altitude_flag == 0 && msg_barometer.vector.x > highest_altitude - 0.5)
        // {
        //     get_to_highest_altitude_flag = 1;
        //     start_land_flag = 1;
        // }
        //下降中：
        // if (start_land_flag == 1)
        // {
        land_target_altitude -= 0.055;
        // }
        //降落完成：
        if (land_target_altitude <= 10.50)
        {
            break;
        }
        // land_complete_cnt++;
        // if (land_complete_cnt >= 150)
        // {
        //     //如果3秒前后高度差大于1,则判断仍在下降
        //     if (abs(msg_barometer.vector.x - last_altitude) > 2)
        //     {
        //         land_complete_cnt = 0;
        //         last_altitude = msg_barometer.vector.x;
        //     }
        //     //否则降落成功：跳出循环
        //     else
        //     {
        //         break;
        //     }
        // }
        //PID部分
        double d_height = land_target_altitude - filtered_height;
        double d_throttle = pid_height.calc(d_height);
        move(0, 0, d_throttle, 0, 5);

        cout << "   target_altitude:" << land_target_altitude << "real_altitude :" << msg_barometer.vector.x << endl;
        usleep(10000);
        ros::spinOnce();
    }
    ROS_INFO("Land complete ");
}

// bool AirsimControl::land()
// {
//     control_client->hover();
//     sleep(1);
//     if (control_client->isApiControlEnabled() == 0)
//     {
//         control_client->enableApiControl(true);
//         ROS_INFO("Command : Enable Api Control");
//     }
//     float takeoffTimeout = 100;
//     control_client->land(takeoffTimeout);
//     sleep(1);
//     control_client->armDisarm(false);
//     ROS_INFO("Command : Land");
// }

bool AirsimControl::move(float pitch, float roll, float throttle, float yaw,
                         float duration)
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->armDisarm(true);
    control_client->moveByAngleThrottle(pitch, roll, throttle, yaw, duration); //ROS_INFO("Command : Move Pitch-%f Roll-%f Throtele-%f Yaw-%f Dur%f",pitch, roll, throttle, yaw,duration);
}

void AirsimControl::search(double &pitch, double &roll, double &yaw)
{
    if (leftright_count > 0) //右
    {
        leftright_count++;
        if (abs(leftright_count) < 40)
        {
            roll = 0.1;
        }
        else if (abs(leftright_count) > 40 && abs(leftright_count) < 70)
        {
            roll = 0.05;
        }
        else
        {
            // pitch = 0.0;
            roll = 0.0;
        }
        if (msg_depth_count.vector.y > 0.3)
        {
            roll = -0.3;
            search_flag = 1;
        }
        else if (search_flag == 1 && msg_depth_count.vector.y > 0 && msg_depth_count.vector.y < msg_pre_depth_count.vector.y && abs(leftright_count) > 50)
        {
            control_client->hover();
            sleep(2);
            leftright_count = -1;
            search_flag = 0;
        }
    }
    else //左
    {
        leftright_count--;
        if (abs(leftright_count) < 50)
        {
            roll = -0.1;
        }
        else if (abs(leftright_count) > 50 && abs(leftright_count) < 80)
        {
            roll = -0.05;
        }
        else
        {
            // pitch = 0.0;
            roll = 0.0;
        }
        if (msg_depth_count.vector.x > 0.3)
        {
            roll = 0.3;
            search_flag = 1;
        }
        else if (search_flag == 1 && msg_depth_count.vector.x > 0 && msg_depth_count.vector.x < msg_pre_depth_count.vector.x && abs(leftright_count) > 50)
        {
            control_client->hover();
            sleep(2);
            leftright_count = 1;
            search_flag = 0;
            ROS_INFO("TURN");
        }
    }
    if (abs(leftright_count)>400)
        leftright_count=-leftright_count/abs(leftright_count);
    ROS_INFO(" SEA %d,%f,%f,%f", leftright_count, msg_depth_count.vector.x, msg_depth_count.vector.y, msg_depth_count.vector.z);

    // if (abs(leftright_count) < 50)
    // {
    //     roll = (leftright_count > 0) ? 0.1 : -0.1;
    // }
    // else if (abs(leftright_count) > 50 && abs(leftright_count) < 80)
    // {
    //     roll = (leftright_count > 0) ? 0.05 : -0.05;
    // }
    // else
    // {
    //     // pitch = 0.0;
    //     roll = 0.0;
    // }

    // if (msg_depth_count > 0.3 && error_code <= pre_error_code)
    // {
    //     // pitch = 0.10;
    //     roll = (leftright_count > 0) ? -0.3 : 0.3;
    // }
    // if (error_code > 240 && error_code <= 255 && error_code > pre_error_code && abs(leftright_count) > 100)
    // {
    //     control_client->hover();
    //     sleep(1);
    //     // roll = (leftright_count > 0) ? 0.1 : -0.1;
    //     leftright_count = (leftright_count > 0) ? -1 : 1;
    // }
    ROS_INFO("%f %f", msg_depth_count.vector.x, msg_depth_count.vector.y);
}

bool AirsimControl::go_forward(double &pitch, double &roll, double &yaw)
{
    yaw = initial_yaw;
    if (abs(tf::getYaw(msg_imu.orientation) - yaw) < 0.02)
    {
        forward_count++;
        if (forward_count < 80)
        {
            pitch = -0.05;
        }
        else if (forward_count > 80 && forward_count < 200)
        {
            pitch = 0.0;
        }
        else if (forward_count > 200 && forward_count < 240)
        {
            pitch = 0.15;
        }
        else
        {
            control_client->hover();
            sleep(2);
            forward_count = 0;
            return 1;
        }
    }
    ROS_ERROR("##############33");
    return 0;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("kp", kp);
    private_nh.getParam("ki", ki);
    private_nh.getParam("kd", kd);
    private_nh.getParam("max", pid_max);
    private_nh.getParam("kp2", kp2);
    private_nh.getParam("ki2", ki2);
    private_nh.getParam("kd2", kd2);
    private_nh.getParam("max2", pid_max2);
    AirsimControl airsim_ctrl;
    sleep(3);
    airsim_ctrl.run();
}

void gostraight()
{
    // PIDctrl pid_height;
    // pid_height.init(0.08, 0.0003, 1.5, 1000000);
    // double d_height = target_height - msg_barometer.vector.x;
    // control_throttle = pid_height.calc(d_height);
    ;
}