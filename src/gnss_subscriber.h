#ifndef GNSS_SUBSCRIBER
#define GNSS_SUBSCRIBER

#include <deque>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "gnss_cal/gnssCal.h"

namespace lidar_localization{

class GNSSdata{
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        int status = 0;
        int service = 0;
    
        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

    private:
        static bool origin_position_inited;
    
    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GNSSdata>& UnsyncedData, std::deque<GNSSdata>& SyncedData, double sync_time);
};



class GNSSSubscriber{
    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber()=default;
        void ParseData(std::deque<GNSSdata>& deque_gnss_data);
    
    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GNSSdata>new_gnss_data_;
        std::mutex buff_mutex_;
};

}

#endif