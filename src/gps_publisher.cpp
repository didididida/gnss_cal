#include "gps_publisher.h"

namespace lidar_localization{

gps_publisher::gps_publisher(ros::NodeHandle &nh, std::string topic_name,std::string frame_id,int buff_size)
:nh_(nh),frame_id(frame_id){
   publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_name,buff_size);
}

void gps_publisher::Publish(GNSSdata gnss_data){
    sensor_msgs::NavSatFix nav;
    ros::Time ros_time((double)gnss_data.time);
    nav.header.stamp=ros_time;
    nav.altitude = gnss_data.altitude;
    nav.latitude = gnss_data.latitude;
    nav.longitude = gnss_data.longitude;
    nav.status.service = gnss_data.service;
    nav.status.status = gnss_data.status;
    publisher_.publish(nav);
}


bool gps_publisher::hasSubscriber(){
    return publisher_.getNumSubscribers()!=0;
}
}