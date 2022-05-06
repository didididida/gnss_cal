#include <ros/ros.h>

#include "satellite_subscriber.h"
#include "plane_subscriber.h"
#include "imu_subscriber.h"
#include "gnss_subscriber.h"

using namespace lidar_localization;

int main(int argc, char *argv[]){

    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    std::shared_ptr<IMUSubscriber>imu_sub_ptr = std::make_shared<IMUSubscriber>(nh,"/imu/data",1000);
    std::shared_ptr<GNSSSubscriber>gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh,"/ublox_driver/receiver_lla",1000);
    std::shared_ptr<PLANEsubscriber>plane_sub_ptr = std::make_shared<PLANEsubscriber>(nh,"/planecheck/planes_coefficient",1000);
    std::shared_ptr<SATSubscriber>sat_sub_ptr = std::make_shared<SATSubscriber>(nh,"/gnss_post_cal",1000);
    
    std::deque<GNSSdata>gnss_data_buff;
    std::deque<IMUData>imu_data_buff;
    std::deque<ALL_plane>plane_data_buff;
    std::deque<ALL_sat>sat_data_buff;

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        gnss_sub_ptr->ParseData(gnss_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        plane_sub_ptr->ParseData(plane_data_buff);
        sat_sub_ptr->ParseData(sat_data_buff);

        while(gnss_data_buff.size()>0&&
              imu_data_buff.size()>0&&
              plane_data_buff.size()>0&&
              sat_data_buff.size()>0)
        {
          
        GNSSdata gnss_data = gnss_data_buff.front();
        IMUData imu_data = imu_data_buff.front();
        ALL_sat all_sat_data = sat_data_buff.front();
        ALL_plane all_plane_data = plane_data_buff.front();

        double d_time = gnss_data.time - imu_data.time;
        gnss_data_buff.pop_front();
        imu_data_buff.pop_front();
        ROS_INFO("````````````%f",d_time);
        ROS_INFO("OKKKKKKKKKK");

        }
    }
    return 0;
}