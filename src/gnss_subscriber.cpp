#include "gnss_subscriber.h"

namespace lidar_localization{

GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
:nh_(nh){
    subscriber_ = nh_.subscribe(topic_name,buff_size,&GNSSSubscriber::msg_callback,this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr){
    buff_mutex_.lock();
    GNSSdata data;
    data.time = nav_sat_fix_ptr->header.stamp.toSec();
    data.latitude = nav_sat_fix_ptr->latitude;
    data.altitude = nav_sat_fix_ptr->altitude;
    data.longitude = nav_sat_fix_ptr->longitude;
    data.status = nav_sat_fix_ptr->status.status;
    data.service = nav_sat_fix_ptr->status.service;
    new_gnss_data_.push_back(data);
    buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSdata>& gnss_data_buff){
    buff_mutex_.lock();
    if(new_gnss_data_.size()>0){
        gnss_data_buff.insert(gnss_data_buff.end(),new_gnss_data_.begin(),new_gnss_data_.end());
        new_gnss_data_.clear();
    }
    buff_mutex_.unlock();
}
}