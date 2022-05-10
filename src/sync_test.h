#ifndef SYNCHRONIZED_DATA
#define SYNCHRONIZED_DATA

#include <ros/ros.h>
#include "satellite_subscriber.h"
#include "plane_subscriber.h"
#include "imu_subscriber.h"
#include "gnss_subscriber.h"
#include "gps_publisher.h"
#include "particle_filter.h"

namespace lidar_localization{

class sync_data{
    public:
    sync_data(ros::NodeHandle& nh, std::string cloud_topic);
    bool run();
    private:
    bool readData();
    bool hasData();
    bool validData();
    
    bool publishData();
    std::shared_ptr<IMUSubscriber>imu_sub_ptr;
    std::shared_ptr<GNSSSubscriber>gnss_sub_ptr;
    std::shared_ptr<PLANEsubscriber>plane_sub_ptr;
    std::shared_ptr<SATSubscriber>sat_sub_ptr;
     

    std::shared_ptr<gps_publisher>gps_pub_ptr;

    std::deque<GNSSdata>gnss_data_buff;
    std::deque<IMUData>imu_data_buff;
    std::deque<ALL_plane>plane_data_buff;
    std::deque<ALL_sat>sat_data_buff;

    GNSSdata current_gnss;
    IMUData current_imu;
    ALL_plane current_planes;
    ALL_sat current_sat;
};
}

#endif