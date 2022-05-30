#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "md/encoder.h"
#include <tf/transform_broadcaster.h>

#define DISTANCE_THRESHOLD 2.0 // (m)
#define ANGLE_THRESHOLD 25 // (degree)

using namespace std;

int encoderInfo[4] = {0, 0, 0, 0}; // left degree, round, right degree, round
long encoderTotal[4] = {0, 0, 0, 0}; // left current, prev, right current, prev
long initialEncoderTotal[2] = {0, 0};
bool initialEncoderTotalSaved[2] = {false, false};
ros::Time currentTime;
ros::Time prevTime;
float wheel_separation = 0.97;
float wheel_radius = 0.23;
float one_rotation_distance = 2 * M_PI * wheel_radius;
double x, y, translation_x, translation_y, yaw, prevYaw, initialYaw;
bool initialYawSaved = false;
bool encoderReceived[2] = {false, false}; // left, right

int current_goal_idx = 1;
int steering_pos[2] = {0, 0}; // current, prev

// float goal_poses[14][2] = { // t.x, t.y
float goal_poses[6][2] = { // t.x, t.y
    {0, 0}, // t.x, t.y
    {20.6499, 1.42222},
    {20.6499, 1.42222},
    {30.8857, 12.0816},
    {30.8857, 12.0816},
    {31.139, 22.868}
    // {90.0, 68.7},
    // {86.0, 66.5},
    // {86.0, 66.5},
    // {90.0, 12.0},
    // {90.0, 12.0},
    // {85.0, 5.0},
    // {85.0, 5.0},
    // {0.0, 0.0}
};
int goal_steering_angles[6] = {0, 0, 362, 362, 0, 0}; // steering
int translate_section_idx[3] = {1,3,5}; 
int rotate_section_idx[2] = {2,4};
// int goal_steering_angles[14] = {0, 0, 720, 720, 0, 0, 1440, 1440, 0, 0, -720, -720, 0, 0}; // steering
// int translate_section_idx[7] = {1,3,5,7,9,11,13}; 
// int rotate_section_idx[6] = {2,4,6,8,10,12}; 
ros::Publisher accel_pub;
ros::Publisher steering_pub;

bool isAutoControlMode = true;

double constrainAngle (double angle) { // constrain the angle between -pi to pi
    int quotient;
    if(angle > M_PI) {
        quotient = floor(angle / M_PI);
        if(quotient % 2 == 0) {
            angle -= quotient * M_PI;
        } else {
            angle -= (quotient - 1) * M_PI;
            angle = angle - 2 * M_PI;
        }
    } else if (angle < -M_PI) {
        quotient = floor(-angle / M_PI);
        if(quotient % 2 == 0) {
            angle += quotient * M_PI;
        } else {
            angle += (quotient - 1) * M_PI;
            angle = angle + 2 * M_PI;
        }
    }
    return angle;
}

tf::Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    tf::Quaternion q;
    q.setW(cr * cp * cy + sr * sp * sy);
    q.setX(sr * cp * cy - cr * sp * sy);
    q.setY(cr * sp * cy + sr * cp * sy);
    q.setZ(cr * cp * sy - sr * sp * cy);

    return q;
}

double qToYaw(geometry_msgs::Quaternion q) {
    double yawTemp;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    // angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    // if (std::abs(sinp) >= 1)
    //     angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    // else
    //     angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yawTemp = std::atan2(siny_cosp, cosy_cosp);

    return yawTemp;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    currentTime = msg->header.stamp;
    double yawTemp = qToYaw(msg->orientation);
    if(!initialYawSaved) {
        initialYaw = yawTemp;
        initialYawSaved = true;
    }
    yaw = yawTemp - initialYaw;
    yaw = constrainAngle(yaw);
    currentTime = msg->header.stamp;
    // std::cout << "yawTemp " << yawTemp << std::endl;
}

void leftEncoderCallback(const md::encoderConstPtr& msg) {
    encoderInfo[0] = msg->Degree;
    encoderInfo[1] = msg->Round;
    if(!initialEncoderTotalSaved[0]) {
        initialEncoderTotal[0] = msg->Round * 360 + msg->Degree;
        initialEncoderTotalSaved[0] = true;
    }
    encoderTotal[0] = msg->Round * 360 + msg->Degree - initialEncoderTotal[0];
    if(!encoderReceived[0]) {
        encoderTotal[1] = encoderTotal[0];
        encoderReceived[0] = true;
    }
}

void rightEncoderCallback(const md::encoderConstPtr& msg) {
    encoderInfo[2] = msg->Degree;
    encoderInfo[3] = msg->Round;
    if(!initialEncoderTotalSaved[1]) {
        initialEncoderTotal[1] = msg->Round * 360 + msg->Degree;
        initialEncoderTotalSaved[1] = true;
    }
    encoderTotal[2] = msg->Round * 360 + msg->Degree - initialEncoderTotal[1];
    if(!encoderReceived[1]) {
        encoderTotal[3] = encoderTotal[2];
        encoderReceived[1] = true;
    }
}

void calculate_tf() {
    // std::cout << encoderTotal[0] << "\t" << encoderTotal[1] << "\t" << encoderTotal[2] << "\t" << encoderTotal[3] << "\t" << std::endl;
    double left_distance = one_rotation_distance * ((double)(encoderTotal[0] - encoderTotal[1]) / 360.0);
    double right_distance = one_rotation_distance * ((double)(encoderTotal[2] - encoderTotal[3]) / 360.0);
    double average_distance = (left_distance + right_distance) / 2.0;
    // double yaw_delta = (right_distance - left_distance) / wheel_separation;
    double yaw_delta = yaw - prevYaw;
    prevYaw = yaw;
    // if(yaw_delta != 0) {
    //     yaw += yaw_delta;
    // }
    if(average_distance != 0) {
        x = average_distance * cos(yaw_delta/2);
        y = -average_distance * sin(yaw_delta/2);
        translation_x += (cos(yaw) * x - sin(yaw) * y);
        translation_y += (sin(yaw) * x + cos(yaw) * y);
    }

    encoderTotal[1] = encoderTotal[0];
    encoderTotal[3] = encoderTotal[2];
}

bool checkCarStopped() {
    if((steering_pos[0] - steering_pos[1]) == 0 && (encoderTotal[0] - encoderTotal[1]) == 0 && (encoderTotal[2] - encoderTotal[3]) == 0) {
        return true;
    } else {
        return false;
    }
}

bool checkReachedThreshold()
{
    calculate_tf();
    int* find_trans = std::find(translate_section_idx,translate_section_idx+3,current_goal_idx);
    int* find_rot = std::find(rotate_section_idx,rotate_section_idx+2,current_goal_idx);
    // int* find_trans = std::find(translate_section_idx,translate_section_idx+7,current_goal_idx);
    // int* find_rot = std::find(rotate_section_idx,rotate_section_idx+6,current_goal_idx);
    if(find_trans != translate_section_idx+3) { // translate section
    // if(find_trans != translate_section_idx+7) { // translate section
        float* current_goal_pose = goal_poses[current_goal_idx];
        float distance = sqrt(pow(current_goal_pose[0] - translation_x, 2) + pow(current_goal_pose[1] - translation_y, 2));
        if(distance <= DISTANCE_THRESHOLD)
        {
            std_msgs::String accel_msg;
            accel_msg.data = "backward";
            accel_pub.publish(accel_msg);
            std::cout << "backward" << std::endl;
            return true;
        }
        else
        {
            std_msgs::String accel_msg;
            accel_msg.data = "forward";
            std::cout << "forward" << std::endl;
            accel_pub.publish(accel_msg);
            return false;
        }
    } else if(find_rot != rotate_section_idx+2) { // rotate section
    // } else if(find_rot != rotate_section_idx+6) { // rotate section
        int current_goal_angle = goal_steering_angles[current_goal_idx];
        int angleDiff = std::abs(current_goal_angle - steering_pos[0]);
        if(angleDiff <= ANGLE_THRESHOLD)
        {
            // std_msgs::Int32 steering_msg;
            // steering_msg.data = 0;
            // steering_pub.publish(steering_msg);
            return true;
        } else {
            std_msgs::Int32 steering_msg;
            if(current_goal_idx == rotate_section_idx[0]) {
                // steering_msg.data = 720;
                steering_msg.data = 362;
            }
            // else if (current_goal_idx == rotate_section_idx[2]) {
            //     steering_msg.data = 1440;
            // } else if (current_goal_idx == rotate_section_idx[4]) {
            //     steering_msg.data = -720;
            // }
            else {
                steering_msg.data = 0;
            }

            steering_pub.publish(steering_msg);
            return false;
        }
    }
}

void goalCheck() {
    calculate_tf();
    string printed_message = "";
    // printed_message += to_string(translation_x) + "\t" + to_string(translation_y) + "\t" + to_string(yaw) + "\t";
    for(int i=0; i<4; i++) {
        float* goal_pose = goal_poses[i];
        double distance_diff = sqrt(pow(goal_pose[0] - translation_x, 2) + pow(goal_pose[1] - translation_y, 2));
        // double yaw_diff = goal_pose[2] - yaw;
        // yaw_diff = constrainAngle(yaw_diff);
        // printed_message += to_string(i) + ": " + to_string(distance_diff) + "\t" + to_string(yaw_diff) + ", ";
        if(distance_diff < DISTANCE_THRESHOLD) {
            printed_message += to_string(i) + ": " + to_string(distance_diff) + ", ";
        }
    }
    if(printed_message != "") {
        std::cout << printed_message << std::endl;
    }
}

void steeringPosCallback(const std_msgs::Int32ConstPtr& msg)
{
    steering_pos[1] = steering_pos[0];
    steering_pos[0] = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_follower");
    ros::NodeHandle n;
    ros::Subscriber left_sub = n.subscribe<md::encoder>("/left_angle", 2, leftEncoderCallback);
    ros::Subscriber right_sub = n.subscribe<md::encoder>("/right_angle", 2, rightEncoderCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 2, imuCallback);
    ros::Subscriber steering_pose_sub = n.subscribe<std_msgs::Int32>("steering_pos_topic", 2, steeringPosCallback);

    accel_pub = n.advertise<std_msgs::String>("/accel_control_topic", 10);
    // steering_pub = n.advertise<std_msgs::String>("/steering_wheel_control_topic", 10);
    steering_pub = n.advertise<std_msgs::Int32>("/cmd_posi", 10);;

    x = 0.0;
    y = 0.0;
    translation_x = 0.0;
    translation_y = 0.0;
    yaw = 0.0;
    prevYaw = 0.0;
    initialYaw = 0.0;

    ros::Rate rate(30);
    while (ros::ok())
    {
        if(encoderReceived[0] && encoderReceived[1]) {
            // goalCheck();
            if(isAutoControlMode) {
                bool isStopped = checkCarStopped();
                bool isReached = checkReachedThreshold();
                if(isReached && isStopped && current_goal_idx < 5) {
                // if(isReached && isStopped && current_goal_idx < 13) {
                    current_goal_idx += 1;
                }
                float* current_goal_pose = goal_poses[current_goal_idx];
                // std::cout << isStopped << "\t" << steering_pos[0] << "\t" << steering_pos[1] << "\t" << encoderTotal[0] << "\t" << encoderTotal[1] << "\t" << encoderTotal[2] << "\t" << encoderTotal[3] << std::endl;
                std::cout << "yaw(" << initialYaw << "\t" << yaw << ")\tgoal:\t" << current_goal_idx << "\t(" << current_goal_pose[0] << ", " << current_goal_pose[1] << ")\t:\t" << translation_x << "\t" << translation_y << "\t" << steering_pos[0] << std::endl;               
            } else {
                calculate_tf();
                std::cout << translation_x << "\t" << translation_y << "\t" << steering_pos[0] << std::endl;  
            }
        }
        // bool isReached = checkReachedThreshold();
        // if(isReached) {
        //     current_goal_idx += 1;
        // }
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
}
