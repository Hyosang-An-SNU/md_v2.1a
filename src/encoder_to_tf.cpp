#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "md/encoder.h"
#include <tf/transform_broadcaster.h>

int encoderInfo[4] = {0, 0, 0, 0}; // left degree, round, right degree, round
long encoderTotal[4] = {0, 0, 0, 0}; // left current, prev, right current, prev
ros::Time currentTime;
ros::Time prevTime;
float wheel_separation = 0.97;
float wheel_radius = 0.23;
float one_rotation_distance = 2 * M_PI * wheel_radius;
double x, y, translation_x, translation_y, yaw, prevYaw, initialYaw;
bool initialYawSaved = false;

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
    // std::cout << "yawTemp " << yawTemp << std::endl;
}

void leftEncoderCallback(const md::encoderConstPtr& msg) {
    encoderInfo[0] = msg->Degree;
    encoderInfo[1] = msg->Round;
    encoderTotal[0] = msg->Round * 360 + msg->Degree;
}

void rightEncoderCallback(const md::encoderConstPtr& msg) {
    encoderInfo[2] = msg->Degree;
    encoderInfo[3] = msg->Round;
    encoderTotal[2] = msg->Round * 360 + msg->Degree;
}


// 100Hz
void calculate_tf() {
    // std::cout << encoderTotal[0] << " " << encoderTotal[1] << " " << encoderTotal[2] << " " << encoderTotal[3] << " " << std::endl;
    double left_distance = one_rotation_distance * ((double)(encoderTotal[0] - encoderTotal[1]) / 360.0);
    double right_distance = one_rotation_distance * ((double)(encoderTotal[2] - encoderTotal[3]) / 360.0);
    double average_distance = (left_distance + right_distance) / 2.0;
    // double yaw_delta = (right_distance - left_distance) / wheel_separation;
    double yaw_delta = yaw - prevYaw;
    prevYaw = yaw;
    if(average_distance != 0) {
        x = average_distance * cos(0.5*yaw_delta);
        y = -average_distance * sin(0.5*yaw_delta);
        translation_x += (cos(yaw) * x - sin(yaw) * y);
        translation_y += (sin(yaw) * x + cos(yaw) * y);
    }
    // if(yaw_delta != 0) {
    //     yaw += yaw_delta;
    // }

    encoderTotal[1] = encoderTotal[0];
    encoderTotal[3] = encoderTotal[2];
}


// 100Hz
void broadcastTF() {
    calculate_tf();
    static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster brMapToOdom;
    tf::Transform transform;
    tf::Quaternion q;
    tf::Vector3 v3;

    q = ToQuaternion(yaw, 0, 0);
    // q = ToQuaternion(yaw * (M_PI / 180.0), 0, 0);
    // std::cout << "yaw " << yaw * (M_PI / 180.0) << std::endl;
    transform.setRotation(q);
    v3.setX(translation_x);
    v3.setY(translation_y);
    v3.setZ(0.0);
    transform.setOrigin(v3);
    // std::cout << currentTime.toNSec() << std::endl;
    br.sendTransform(tf::StampedTransform(transform, currentTime, "odom", "velodyne"));

    //tf::Transform transformMapToOdom;
    //tf::Quaternion qMapToOdom;
    //tf::Vector3 v3MapToOdom;

    //qMapToOdom = ToQuaternion(0, 0, 0);
    //transformMapToOdom.setRotation(qMapToOdom);
    //v3MapToOdom.setX(0.0);
    //v3MapToOdom.setY(0.0);
    //v3MapToOdom.setZ(0.0);
    //transformMapToOdom.setOrigin(v3MapToOdom);

    //brMapToOdom.sendTransform(tf::StampedTransform(transformMapToOdom, currentTime, "map", "odom"));
    
    prevTime = currentTime;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_to_tf");
    ros::NodeHandle n;
    ros::Subscriber left_sub = n.subscribe<md::encoder>("/left_angle", 2, leftEncoderCallback);
    ros::Subscriber right_sub = n.subscribe<md::encoder>("/right_angle", 2, rightEncoderCallback);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 2, imuCallback);

    x = 0.0;
    y = 0.0;
    translation_x = 0.0;
    translation_y = 0.0;
    yaw = 0.0;
    prevYaw = 0.0;
    initialYaw = 0.0;

    currentTime = ros::Time::now();
    prevTime = currentTime;
    ros::Rate rate(100);
    while (ros::ok())
    {
        // pub.publish(pcMsg);
        broadcastTF();
        // std::cout << x << " " << y << " " << translation_x << " " << translation_y << " " << yaw << std::endl;
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
}
