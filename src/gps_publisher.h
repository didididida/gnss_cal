#ifndef GPS_PUBLISHER_
#define GPS_PUBLISHER_
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "gnss_subscriber.h"
#include <string>

namespace lidar_localization{

class gps_publisher{
    public:
    gps_publisher(ros::NodeHandle &nh, std::string topic_name,std::string frame_id,int buff_size);
    gps_publisher() = default;

    void Publish(GNSSdata gnss_data);
    bool hasSubscriber();

    private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id = "";
};
}
#endif