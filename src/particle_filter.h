/*particle filter class*/

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>
#include "helper_function.h"
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"
#include <sensor_msgs/NavSatFix.h>
#include <mutex>
#include <shared_mutex>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <unordered_map>
#include <random>

class Particle{
public:
    Particle(){};
    int id;
    Eigen::Vector3d p_ecef;
    double weight;
};

struct Pos_info{
    uint64_t pos_time;
    double lat;
    double lon;
    double alt;
};


class Sat_info{
public:
    int id;
    double CN0;
    double psr;
    double ecefX;
    double ecefY;
    double ecefZ;
    double mp;
};

struct Coefficient{
    double a;
    double b;
    double c;
    double d;
};


struct detected_planes{
    uint64_t detect_time;
    std::vector<Coefficient> planes;
};


struct SatelliteInfo{
    uint64_t sat_time;
    std::vector<Sat_info> sat_pos;
    double lat;
    double lon;
    double alt;
};


class ParticleFilter{
public:
     
    /*constructor*/
    ParticleFilter(ros::NodeHandle nh):_nh(nh){
        init_ros();
    }
    void init_ros();
 
    void spin(){
        ros::spin();
    }
    
    /*deconstructor*/
    ~ParticleFilter(){}
    
    /*initialize particles*/
    void init_pf();

    /*update weight according to observation*/
    void updateWeights(double lat,double lon,double alt, Eigen::Matrix<float,4,1> q,std::vector<Plane>planes
    , std::vector<Sat_info>sat);
    void updateWeights();

    /*scatter particles around gps pos*/
    void scatter(const Eigen::Vector3d& pos_lla);
    
    /* initialized Returns whether particle filter is initialized yet or not.*/
    const bool initialized() const {
        return is_initialized;
    }
   
    /*to calculate average result from all particles*/
    Eigen::Vector3d getAverage(const std::vector<Particle> &particles);
    /*update surroundings' planes*/
    void updateLidar (const gnss_cal::detect_planesConstPtr &plane_msg);
    /*update attitude*/
    void updateImu (const sensor_msgs::ImuConstPtr &imu_msg);
    /*update satellites position info*/
    void updateSat (const gnss_cal::gnssCalConstPtr &gps_msg);
    //get gps position
    void updateGps(const sensor_msgs::NavSatFixConstPtr &pos_msg);
   
private:
    ros::NodeHandle _nh;
    ros::Publisher _pub_nav;
    ros::Subscriber _sub_imu;
    ros::Subscriber _sub_lidar;
    ros::Subscriber _sub_gps;
    ros::Subscriber _sub_sat;
    //numbers of particles
    int num_particles;
    std::vector<Particle> particles;
    double lat=0.0,lon=0.0,alt=0.0;
    //Flag 
    mutable std::shared_mutex shMutex;
    bool is_gps_update = false;
    bool is_lidar_update = false;
    bool is_imu_update = false;
    bool is_sat_update = true;
    bool is_initialized =false;
    std::vector<Plane> planes;
    std::vector<Sat_info> sat;
    Eigen::Matrix<float,3,3> C_N2B;
    // variation of pseudorange's error
    double SIGMA_P = 0.5;

    //quoternion
    Eigen::Matrix<float,4,1> q;
   
    std::vector<double>grid={-6.0,-4.0,-2.0,0.0,2.0,4.0,6.0};
    std::vector<Eigen::Vector3d> enu_p;
    //NavSatFix restore
    sensor_msgs::NavSatFix restore_;
    
};

#endif