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

struct Particle{
    int id;
    double x;
    double y;
    double z;
    double weight;
};

struct Pos_info{
    uint64_t pos_time;
    double lat;
    double lon;
    double alt;
};
using gpsPosDataPtr = std::shared_ptr<Pos_info>;

class Sat_info{
public:
    double CN0;
    double psr;
    double ecefX;
    double ecefY;
    double ecefZ;
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
using detectPlanePtr = std::shared_ptr<detected_planes>;

struct SatelliteInfo{
    uint64_t sat_time;
    std::vector<Sat_info> sat_pos;
};


class ParticleFilter{
public:
     
    

    /*constructor*/
    ParticleFilter(){}

    /*deconstructor*/
    ~ParticleFilter(){}
    
    /*initialize*/
    void init(const double &lat,const double &lon,const double &alt);

    /*update weight according to observation*/
    void updateWeights(double lat,double lon,double alt,Eigen::Matrix<float,3,3>cn2b,std::vector<Plane>planes
    , std::vector<Sat_info>sat);
  
    void updateWeights();
    /*predict the next timestamp*/
    void prediction();
    

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
    //numbers of particles
    int num_particles;
    std::vector<Particle> particles;
    detectPlanePtr pPlane;
    gpsPosDataPtr pGps;
    double lat=0.0,lon=0.0,alt=0.0;
    //Flag 
    mutable std::shared_timed_mutex shMutex;
    bool is_gps_update = false;
    bool is_lidar_update = false;
    bool is_imu_update = false;
    bool is_sat_update = true;
    bool is_initialized =false;
    std::vector<Plane> planes;
    std::vector<Sat_info> sat;
    Eigen::Matrix<float,3,3> C_N2B;
};

#endif