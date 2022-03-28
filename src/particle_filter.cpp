#include "particle_filter.h"

void ParticleFilter::init(){
    for(int i = 0;i<num_particles;++i){
        Particle p;
        particles.push_back(p);
    }
    is_initialized = true;
}


void ParticleFilter::scatter(const double &lat,const double &lon,const double&alt){

}


void ParticleFilter::updateWeights(double lat,double lon,double alt,Eigen::Matrix<float,3,3>cn2b,std::vector<Plane>planes
    , std::vector<Sat_info>sat){
    if(!initialized()){
        init();
    }
    scatter(lat,lon,alt);
    /*for each particle do ray-tracing process*/
    for (const auto &p :particles){

    }
}

void ParticleFilter::updateWeights(){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    updateWeights(lat,lon,alt,C_N2B,planes,sat);
}

//update the Plane infomation
void ParticleFilter::updateLidar(const gnss_cal::detect_planesConstPtr &plane_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    planes.clear();
    for(auto it : plane_msg->Coeff){
        Plane p(it.a,it.b,it.c,it.d,it.z_max,it.z_min);
        planes.push_back(p);
    }
    is_lidar_update = true;
    ROS_INFO("LIDAR INFOMATION UPDATE");
}

//update the satellite positio infomation
void ParticleFilter::updateSat (const gnss_cal::gnssCalConstPtr &gps_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    sat.clear();
    for(auto &it:gps_msg->meas){
        Sat_info s;
        s.CN0 = it.CN0.at(0);
        s.ecefX = it.ecefX;
        s.ecefY =it.ecefY;
        s.ecefZ = it.ecefZ;
        s.psr = it.psr;
        sat.push_back(s);
    }
    is_sat_update = true;
    ROS_INFO("GPS LOCATION UPDATE");
}

void ParticleFilter::updateGps(const sensor_msgs::NavSatFixConstPtr &pos_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    lat = pos_msg->latitude;
    lon = pos_msg->longitude;
    alt = pos_msg->altitude;
    is_gps_update = true;
}

//From imu's quaternion to get matrix from enu to body frame
void ParticleFilter::updateImu(const sensor_msgs::ImuConstPtr &imu_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    Eigen::Matrix<float,4,1> q;
    q(0,0) = imu_msg->orientation.w;
    q(1,0) = imu_msg->orientation.x;
    q(2,0) = imu_msg->orientation.y;
    q(3,0) = imu_msg->orientation.z;
    C_N2B = quat2dcm(q);
    is_gps_update = true;
    if(is_gps_update&&is_imu_update&&is_lidar_update&&is_sat_update){
        updateWeights();
    }else{
        return;
    }
}