#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include "rosbag_reader/DataSender.h"
#include "rosbag_reader/TcpServer.h"

// Define server details
const std::string SERVER_IP = "127.0.0.1";
const int SERVER_PORT = 8000;

// Create a DataSender object
DataSender dataSender(SERVER_IP, SERVER_PORT);

// Callback function for GPS messages
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::string data = "GPS: [Latitude: " + std::to_string(msg->latitude) +
                       ", Longitude: " + std::to_string(msg->longitude) +
                       ", Altitude: " + std::to_string(msg->altitude) + "]";
    ROS_INFO("%s", data.c_str());
    dataSender.sendData(data);
}

// Callback function for Compressed Image messages
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg, const std::string& camera_id) {
    std::string data = camera_id + " Image received with size: " + std::to_string(msg->data.size());
    ROS_INFO("%s", data.c_str());
    dataSender.sendData(data);
}

// Callback function for IMU messages
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::string data = "IMU: [Orientation: (" + std::to_string(msg->orientation.x) + ", " +
                       std::to_string(msg->orientation.y) + ", " + std::to_string(msg->orientation.z) + "), " +
                       "Angular velocity: (" + std::to_string(msg->angular_velocity.x) + ", " +
                       std::to_string(msg->angular_velocity.y) + ", " + std::to_string(msg->angular_velocity.z) + "), " +
                       "Linear acceleration: (" + std::to_string(msg->linear_acceleration.x) + ", " +
                       std::to_string(msg->linear_acceleration.y) + ", " + std::to_string(msg->linear_acceleration.z) + ")]";
    ROS_INFO("%s", data.c_str());
    dataSender.sendData(data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_reader");
    ros::NodeHandle nh;

    // Subscribe to sensor topics
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 1000, gpsCallback);
    ros::Subscriber cam1_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_array/cam1/image_raw/compressed", 1000, boost::bind(imageCallback, _1, "cam1"));
    ros::Subscriber cam2_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_array/cam2/image_raw/compressed", 1000, boost::bind(imageCallback, _1, "cam2"));
    ros::Subscriber cam3_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_array/cam3/image_raw/compressed", 1000, boost::bind(imageCallback, _1, "cam3"));
    ros::Subscriber cam4_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_array/cam4/image_raw/compressed", 1000, boost::bind(imageCallback, _1, "cam4"));
    ros::Subscriber cam5_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_array/cam5/image_raw/compressed", 1000, boost::bind(imageCallback, _1, "cam5"));
    ros::Subscriber imu_sub = nh.subscribe("/imu_raw", 1000, imuCallback);

    ros::spin();

    return 0;
}
