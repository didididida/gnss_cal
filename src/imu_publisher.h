#ifndef IMU_PUBLISHER_
#define GPS_PUBLISHER_
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "imu_subscriber.h"
#include <string>

namespace lidar_localization{
class imu_publisher{

public:
    imu_publisher(ros::NodeHandle &nh, std::string topic_name,std::string frame_id,int buff_size);
    imu_publisher() = default;

    void Publish(IMUData imu_data);
    bool hasSubscriber();

    private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id = "";



};
}
#endif