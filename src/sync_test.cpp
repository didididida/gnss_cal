#include "sync_test.h"

namespace lidar_localization{

sync_data::sync_data(ros::NodeHandle& nh, std::string cloud_topic){

    imu_sub_ptr = std::make_shared<IMUSubscriber>(nh,"/imu/data",10000);
    gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh,"/ublox_driver/receiver_lla",10000);
    plane_sub_ptr = std::make_shared<PLANEsubscriber>(nh,"/planecheck/planes_coefficient",10000);
    sat_sub_ptr = std::make_shared<SATSubscriber>(nh,"/gnss_post_cal",10000);
   
    gps_pub_ptr = std::make_shared<gps_publisher>(nh,"/post_gps","aft_mapped",100);
    imu_pub_ptr = std::make_shared<imu_publisher>(nh,"/post_imu","aft_mapped",100);
}

bool sync_data::run(){
   if(!readData())
   return false;
 
   while(hasData()){
      if(!validData())
      continue;
      publishData();
      //std::cout<<"------publish-ok-----"<<std::endl;
      //do something
    }
    return true;
   }

bool sync_data::readData(){
    
    plane_sub_ptr->ParseData(plane_data_buff);
    
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSdata>unsynced_gnss_;
    static std::deque<ALL_sat>unsynced_sat_;
    
    imu_sub_ptr->ParseData(unsynced_imu_);
    sat_sub_ptr->ParseData(unsynced_sat_);
    gnss_sub_ptr->ParseData(unsynced_gnss_);
    
    while(!plane_data_buff.empty()&&plane_data_buff.front().planes.size()<3)
    {
        plane_data_buff.pop_front();
    }

    if(plane_data_buff.size()==0)
    return false;
    
    double cloud_time = plane_data_buff.front().time;
    
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff, cloud_time);
    bool valid_gnss = GNSSdata::SyncData(unsynced_gnss_, gnss_data_buff, cloud_time);
    bool valid_sat = ALL_sat::SyncData(unsynced_sat_,sat_data_buff,cloud_time);
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_sat || !valid_gnss) {
            plane_data_buff.pop_front();
            return false;
        }
        sensor_inited = true;
    }
    
    return true;

}

bool sync_data::hasData(){
    if (plane_data_buff.size() == 0)
        return false;
    if (imu_data_buff.size() == 0)
        return false;
    if (sat_data_buff.size() == 0)
        return false;
    if (gnss_data_buff.size() == 0)
        return false;

    return true;
}


bool sync_data::validData(){
     current_gnss = gnss_data_buff.front();
     current_sat = sat_data_buff.front();
     current_imu = imu_data_buff.front();
     current_planes = plane_data_buff.front();
  
   

     gnss_data_buff.pop_front();
     sat_data_buff.pop_front();
     imu_data_buff.pop_front();
     plane_data_buff.pop_front();

     return true;
}


bool sync_data::publishData(){
      Eigen::Matrix<float,4,1> q;
        q(0,0) = current_imu.orientation.w;
        q(1,0) = current_imu.orientation.x;
        q(2,0) = current_imu.orientation.y;
        q(3,0) = current_imu.orientation.z;

        Eigen::Vector3d pos_lla;
        pos_lla(0) = current_gnss.latitude;
        pos_lla(1) = current_gnss.longitude;
        pos_lla(2) = current_gnss.altitude;
        ParticleFilter PF(q,pos_lla);
    
        Eigen::Vector3d post_pos;
        post_pos = PF.updateWeights(current_planes,current_sat);
        
        //initial position
        Eigen::Vector3d ecef_ref{4018477,425661,4918434};
        Eigen::Vector3d lla_ref=gnss_comm::ecef2geo(ecef_ref);
        Eigen::Vector3d ecef = gnss_comm::geo2ecef(post_pos);
        ecef_ref=ecef_ref-ecef;
        Eigen::Vector3d enu;
        enu = gnss_comm::ecef2enu(lla_ref,ecef_ref);
        //std::cout<<enu[0]<<std::endl;

        GNSSdata data_pub;
        data_pub.altitude = post_pos[2];
        data_pub.longitude = post_pos[1];
        data_pub.latitude = post_pos[0];
        data_pub.service = current_gnss.service;
        data_pub.status = current_gnss.status;
        data_pub.time = current_gnss.time;
        gps_pub_ptr->Publish(data_pub);
        
        //imu_pub_ptr->Publish(current_imu);
}
}
