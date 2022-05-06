#ifndef SATELLITE_SUBSCRIBER
#define SATELLITE_SUBSCRIBER

#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <deque>
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"

namespace lidar_localization{

class Single_sat{
    public:
    int id = 0;
    double CN0 = 0.0; 
    double psr = 0.0;
    double mp = 0.0;
    double ecefX = 0.0;
    double ecefY = 0.0;
    double ecefZ = 0.0;
};

class ALL_sat{
    public:
    double time = 0.0;
    std::vector<Single_sat> sats;
    static bool SyncData(std::deque<ALL_sat>& UnsyncedData, std::deque<ALL_sat>& SyncedData, double sync_time);
};

class SATSubscriber{
    public:
    SATSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    SATSubscriber()=default;
    void ParseData(std::deque<ALL_sat>& deque_sat_data);
    private:
    void msg_callback(const gnss_cal::gnssCalConstPtr& sat_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<ALL_sat>new_sat_data_;
    std::mutex buff_mutex_;

};
}
#endif