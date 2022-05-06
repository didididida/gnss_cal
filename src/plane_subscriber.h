#ifndef PLANE_SUBSCRIBER
#define PLANE_SUBSCRIBER

#include  <ros/ros.h>
#include  <mutex>
#include  <thread>
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"
#include <deque>

namespace lidar_localization{

class Single_plane{
    public:
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
    double z_min = 0.0;
    double z_max = 0.0;
};

class ALL_plane{
    public:
    double time = 0.0;
    std::vector<Single_plane>planes;
    static bool SyncData(std::deque<ALL_plane>& UnsyncedData, std::deque<ALL_plane>& SyncedData, double sync_time);
};

class PLANEsubscriber{
    public:
    PLANEsubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    PLANEsubscriber()=default;
    void ParseData(std::deque<ALL_plane>& deque_plane_data);
    private:
    void msg_callback(const gnss_cal::detect_planesConstPtr&plane_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<ALL_plane>new_plane_data_;
    std::mutex buff_mutex_;
};

}
#endif