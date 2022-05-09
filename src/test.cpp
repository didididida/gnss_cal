#include <ros/ros.h>
#include "imu_subscriber.h"
#include "gnss_subscriber.h"
#include "particle_filter.h"
#include "gps_publisher.h"

using namespace lidar_localization;

int main(int argc, char *argv[]){

    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    std::shared_ptr<IMUSubscriber>imu_sub_ptr = std::make_shared<IMUSubscriber>(nh,"/imu/data",10000);
    std::shared_ptr<GNSSSubscriber>gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh,"/ublox_driver/receiver_lla",10000);
    std::shared_ptr<PLANEsubscriber>plane_sub_ptr = std::make_shared<PLANEsubscriber>(nh,"/planecheck/planes_coefficient",10000);
    std::shared_ptr<SATSubscriber>sat_sub_ptr = std::make_shared<SATSubscriber>(nh,"/gnss_post_cal",10000);
    
    std::shared_ptr<gps_publisher>gps_pub_ptr = std::make_shared<gps_publisher>(nh,"/post_gps","aft_mapped",100);


    std::deque<GNSSdata>gnss_data_buff;
    std::deque<IMUData>imu_data_buff;
    std::deque<ALL_plane>plane_data_buff;
    std::deque<ALL_sat>sat_data_buff;

    ros::Rate rate(10);
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

        //std::cout<<all_plane_data.planes.size()<<std::endl;
        //std::cout<<std::setprecision(10)<<gnss_data.latitude<<"---"<<gnss_data.longitude<<std::endl;
        //std::cout<<all_sat_data.sats[0].ecefX<<std::endl;
        double d_time = gnss_data.time - imu_data.time;
        gnss_data_buff.pop_front();
        imu_data_buff.pop_front();
        plane_data_buff.pop_front();
        sat_data_buff.pop_front();
        
        
        //quoternion
        Eigen::Matrix<float,4,1> q;
        q(0,0) = imu_data.orientation.w;
        q(1,0) = imu_data.orientation.x;
        q(2,0) = imu_data.orientation.y;
        q(3,0) = imu_data.orientation.z;

        Eigen::Vector3d pos_lla;
        pos_lla(0) = gnss_data.latitude;
        pos_lla(1) = gnss_data.longitude;
        pos_lla(2) = gnss_data.altitude;
        
        ParticleFilter PF(q,pos_lla);
    
        Eigen::Vector3d post_pos;
        post_pos = PF.updateWeights(all_plane_data,all_sat_data);
        
        GNSSdata data_pub;
        data_pub.altitude = post_pos[2];
        data_pub.longitude = post_pos[1];
        data_pub.latitude = post_pos[0];
        data_pub.service = gnss_data.service;
        data_pub.status = gnss_data.status;
        data_pub.time = gnss_data.time;
        gps_pub_ptr->Publish(data_pub);
        //std::cout<<std::setprecision(9)<<post_pos[0]<<"---"<<post_pos[1]<<std::endl;;
       }
    } 
    return 0;
}