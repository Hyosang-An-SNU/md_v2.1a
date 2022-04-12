// 바퀴 반지름 (m)
#define WHEEL_RADIUS 0.23
// 최대속도 (km/h)
#define MAX_SPEED 6
#define PID_MAX_VAL 128
#define EASY_CTRL_PID 42

// 루프 hz
#define loop_hz 10

#include <ros/ros.h>
#include <ros_essentials_cpp/encoder.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cmath>
#include <iostream>

// encoder
int left_degree;
int left_round;

int right_degree;
int right_round;

int prev_left_degree;
int prev_left_round;
int prev_right_degree;
int prev_right_round;

float left_degree_delta;
float right_degree_delta;

// speed (km/h)
float target_speed;
float present_speed;

// PID
float Kp = 25. / static_cast<float>(MAX_SPEED);
float Ki = 15;
float Kd = 0.8;

float e;
float e_P;
float e_I;
float e_D;
float prev_e;

short PID_val;

// fwd rev flag
bool fwd = true;

// easy control flag
bool easy_ctrl_flag = false;

std::string fwd_rev_indicator;

void left_encoder_callback(const ros_essentials_cpp::encoder::ConstPtr &msg)
{
    left_degree = msg->Degree;
    left_round = msg->Round;
}

void right_encoder_callback(const ros_essentials_cpp::encoder::ConstPtr &msg)
{
    right_degree = msg->Degree;
    right_round = msg->Round;
}

void accel_callback(const std_msgs::String::ConstPtr &msg)
{
    // normal mode
    if (easy_ctrl_flag == false)
    {
        if (fwd == true) // 전진
        {
            if (msg->data == "foward")
            {
                target_speed += 0.1;
                if (target_speed > MAX_SPEED)
                {
                    target_speed = MAX_SPEED;
                }
            }

            else if (msg->data == "backward")
            {
                target_speed -= 0.1;
                if (target_speed < 0)
                {
                    target_speed = 0;
                }
            }

            else if (msg->data == "brake")
            {
                target_speed = 0;
                e_I = 0;
                PID_val = 0;
            }
        }

        else //후진
        {
            if (msg->data == "foward")
            {
                target_speed += 0.05;
                if (target_speed > 0)
                {
                    target_speed = 0;
                }
            }

            else if (msg->data == "backward")
            {
                target_speed -= 0.05;
                if (target_speed < -MAX_SPEED)
                {
                    target_speed = -MAX_SPEED;
                }
            }

            else if (msg->data == "brake")
            {
                target_speed = 0;
                e_I = 0;
                PID_val = 0;
            }
        }

        if (target_speed > 0 && target_speed <= 1)
        {
            target_speed = 1;
        }
    }

    // easy control mode
    else
    {
        if (fwd == true) // 전진 모드
        {
            if (msg->data == "foward")
            {
                PID_val = EASY_CTRL_PID;
            }

            else
            {
                PID_val = 0;
            }
        }

        else //후진 모드
        {
            if (msg->data == "backward")
            {
                PID_val = EASY_CTRL_PID;
            }

            else
            {
                PID_val = 0;
            }
        }
    }
}

void fwdrev_callback(const std_msgs::Empty::ConstPtr &msg)
{
    fwd = !fwd;
    target_speed = 0;
    e_I = 0;
    PID_val = 0;
}

void easy_ctrl_mode_callback(const std_msgs::Empty::ConstPtr &msg)
{
    easy_ctrl_flag = !easy_ctrl_flag;
    target_speed = 0;
    e_I = 0;
    PID_val = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accel_pid_node");
    // create a node handle: it is reference assigned to a new node
    ros::NodeHandle nh;

    ros::Subscriber left_sub = nh.subscribe("left_angle", 1, left_encoder_callback);
    ros::Subscriber right_sub = nh.subscribe("right_angle", 1, right_encoder_callback);
    ros::Subscriber accel_sub = nh.subscribe("accel_control_topic", 1, accel_callback);
    ros::Subscriber fwdrev_sub = nh.subscribe("fwdrev_topic", 1, fwdrev_callback);
    ros::Subscriber easy_ctrl_mode_sub = nh.subscribe("easy_ctrl_mode_topic", 1, easy_ctrl_mode_callback);
    ros::Publisher accel_pid_val_pub = nh.advertise<std_msgs::Int16>("accel_pid_topic", 1);

    ros::Rate r(loop_hz); // loop_hz = 10

    float t_delta = 1. / loop_hz;
    std_msgs::Int16 pid_val_msg;
    ros::spinOnce();

    ros::Duration(2).sleep();
    ros::spinOnce();

    prev_left_degree = left_degree;
    prev_left_round = left_round;
    prev_right_degree = right_degree;
    prev_right_round = right_round;

    e = 0;
    e_P = 0;
    e_I = 0;
    e_D = 0;
    prev_e = 0;

    PID_val = 0;

    present_speed = 0;
    target_speed = 0;

    while (ros::ok())
    {
        // normal mode
        if (easy_ctrl_flag == false)
        {
            ros::spinOnce();

            left_degree_delta = ((left_round - prev_left_round) * 2 * M_PI) + ((left_degree - prev_left_degree) * M_PI / 180.);
            right_degree_delta = ((right_round - prev_right_round) * 2 * M_PI) + ((right_degree - prev_right_degree) * M_PI / 180.);

            prev_left_degree = left_degree;
            prev_left_round = left_round;
            prev_right_degree = right_degree;
            prev_right_round = right_round;

            present_speed = (present_speed + (3.6 * ((left_degree_delta + right_degree_delta) / 2.) * WHEEL_RADIUS / t_delta)) / 2; // 3.6은 m/s -> km/h 변환, 직전의 속도와 평균

            e = target_speed - present_speed;

            e_P = Kp * e;
            e_I = e_I + (Ki * e * t_delta);

            //적분항 누적 크기 제한
            if (e_I > 255)
            {
                e_I = 255;
            }

            e_D = Kd * (e - prev_e) / t_delta;

            prev_e = e;

            PID_val = static_cast<short>(e_P + e_I + e_D);

            // 후진의 케이스
            if (fwd == false)
            {
                PID_val = -PID_val;
            }

            if (PID_val <= 0)
                PID_val = 0;
            if (PID_val >= 255)
                PID_val = 255;

            // 1~28의 pid 값을 28로 끌어올려줘서 안정성 향상
            if (PID_val >= 1 && PID_val <= 28)
            {
                PID_val = 28;
            }

            if (PID_val > PID_MAX_VAL)
            {
                PID_val = PID_MAX_VAL;
            }

            // 처음 출발할 때 출렁거리는 현상 방지
            if (target_speed > 0 && target_speed <= 1.7)
            {
                PID_val = 28;
            }

            pid_val_msg.data = PID_val;
            accel_pid_val_pub.publish(pid_val_msg);

            std::cout << std::fixed;
            std::cout.precision(2);
            if (fwd == true)
            {
                fwd_rev_indicator = "[FWD]";
            }
            else
            {
                fwd_rev_indicator = "[REV]";
            }
            std::cout << "[NORMAL MODE] " << fwd_rev_indicator << "\ttarget speed : " << target_speed << "\tpresent speed : " << present_speed << "\tPID VAL : " << PID_val << "   " << e_P << " " << e_I << " " << e_D << std::endl;

            r.sleep();
        }

        // easy control mode
        else
        {
            ros::spinOnce();

            left_degree_delta = ((left_round - prev_left_round) * 2 * M_PI) + ((left_degree - prev_left_degree) * M_PI / 180.);
            right_degree_delta = ((right_round - prev_right_round) * 2 * M_PI) + ((right_degree - prev_right_degree) * M_PI / 180.);

            prev_left_degree = left_degree;
            prev_left_round = left_round;
            prev_right_degree = right_degree;
            prev_right_round = right_round;

            present_speed = (present_speed + (3.6 * ((left_degree_delta + right_degree_delta) / 2.) * WHEEL_RADIUS / t_delta)) / 2; // 3.6은 m/s -> km/h 변환, 직전의 속도와 평균

            pid_val_msg.data = PID_val;
            accel_pid_val_pub.publish(pid_val_msg);

            std::cout << std::fixed;
            std::cout.precision(2);
            if (fwd == true)
            {
                fwd_rev_indicator = "[FWD]";
            }
            else
            {
                fwd_rev_indicator = "[REV]";
            }
            std::cout << "[EASY CONTROL MODE] " << fwd_rev_indicator << "\tpresent speed : " << present_speed << "\tPID VAL : " << PID_val << std::endl;

            r.sleep();
        }
    }

    return 0;
}