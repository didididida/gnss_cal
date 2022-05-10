#include "imu_subscriber.h"

namespace lidar_localization{

    IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size):nh_(nh){
       subscriber_ = nh_.subscribe(topic_name,buff_size,&IMUSubscriber::msg_callback,this);
    }

    void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr){
        buff_mutex_.lock();
        IMUData imu_data;
        imu_data.time = imu_msg_ptr->header.stamp.toSec();

        imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
        imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
        imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

        imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
        imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
        imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

        imu_data.orientation.x = imu_msg_ptr->orientation.x;
        imu_data.orientation.y = imu_msg_ptr->orientation.y;
        imu_data.orientation.z = imu_msg_ptr->orientation.z;
        imu_data.orientation.w = imu_msg_ptr->orientation.w;

        new_imu_data_.push_back(imu_data);
        buff_mutex_.unlock();
    }

    void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff){
        buff_mutex_.lock();
        if(new_imu_data_.size()>0){
            imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
            new_imu_data_.clear();
        }
        buff_mutex_.unlock();
    }


    bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time)
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
    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;

    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
 
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;

    //normalize quoternion
    synced_data.orientation.Normlize();

    SyncedData.push_back(synced_data);

    return true;

    }
}