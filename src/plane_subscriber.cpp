#include "plane_subscriber.h"

namespace lidar_localization{

PLANEsubscriber::PLANEsubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size):nh_(nh){
    subscriber_ = nh_.subscribe(topic_name,buff_size,&PLANEsubscriber::msg_callback,this);
}

void PLANEsubscriber::msg_callback(const gnss_cal::detect_planesConstPtr& planes_ptr){
    buff_mutex_.lock();
    ALL_plane all;
    all.time = planes_ptr->header.stamp.toSec();
    for(auto i :planes_ptr->Coeff){
        Single_plane sp;
        sp.a = i.a;
        sp.b = i.b;
        sp.c = i.c;
        sp.d = i.d;
        sp.z_min = i.z_min;
        sp.z_max = i.z_max;
        all.planes.push_back(sp);
    }
    new_plane_data_.push_back(all);
    buff_mutex_.unlock();
}

void PLANEsubscriber::ParseData(std::deque<ALL_plane>& deque_plane_data){
    buff_mutex_.lock();
    if(new_plane_data_.size()>0){
        deque_plane_data.insert(deque_plane_data.end(),new_plane_data_.begin(),new_plane_data_.end());
        new_plane_data_.clear();
    }
    buff_mutex_.unlock();
}


}