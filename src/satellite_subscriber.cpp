#include "satellite_subscriber.h"

namespace lidar_localization{
    SATSubscriber::SATSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name,buff_size,&SATSubscriber::msg_callback,this);
    }
    
    void SATSubscriber::msg_callback(const gnss_cal::gnssCalConstPtr& sat_ptr){
        buff_mutex_.lock();
        ALL_sat all_sat_info;
        all_sat_info.time = sat_ptr->header.stamp.toSec();
        for(auto i:sat_ptr->meas){
            Single_sat ss;
            ss.CN0 = i.CN0[0];
            ss.ecefX = i.ecefX;
            ss.ecefY = i.ecefY;
            ss.ecefZ = i.ecefZ;
            ss.id = i.sat;
            ss.mp = i.mp;
            ss.psr = i.psr;
            all_sat_info.sats.push_back(ss);
        }
        new_sat_data_.push_back(all_sat_info);
        buff_mutex_.unlock();
    }
    
    void SATSubscriber::ParseData(std::deque<ALL_sat>& deque_sat_data){
        buff_mutex_.lock();
        if(new_sat_data_.size()>0){
        deque_sat_data.insert(deque_sat_data.end(),new_sat_data_.begin(),new_sat_data_.end());
        new_sat_data_.clear();
        }
        buff_mutex_.unlock();
    }
}