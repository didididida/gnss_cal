#include "gnss_subscriber.h"

namespace lidar_localization{

//synchronize gnss with lidar
bool GNSSdata::SyncData(std::deque<GNSSdata>& UnsyncedData, std::deque<GNSSdata>& SyncedData, double sync_time)
{
    while(UnsyncedData.size()>=2){
        if(UnsyncedData.front().time >sync_time)
        return false;
        if(UnsyncedData.at(1).time <sync_time){
            UnsyncedData.pop_front();
            continue;
        }
        if(sync_time - UnsyncedData.front().time > 0.2){
            UnsyncedData.pop_front();
            break;
        }
        if(UnsyncedData.at(1).time  - sync_time > 0.2){
            UnsyncedData.pop_front();
            break;
        }
        break;
    }

    
    if(UnsyncedData.size()<2)
    return false;
    GNSSdata front_data = UnsyncedData.at(0);
    GNSSdata back_data = UnsyncedData.at(1);
    GNSSdata synced_data;
    
    double front_scale = (back_data.time-sync_time)/(back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time)/(back_data.time - front_data.time);
    
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    
    SyncedData.push_back(synced_data);
    return true;
}


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