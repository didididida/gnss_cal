/*particle filter class*/

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>
#include "helper_function.h"
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"
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

struct Sat_info{
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
    Coefficient planes[];
};

struct SatelliteInfo{
    uint64_t sat_time;
    Sat_info meas[];
};


class ParticleFilter{
public:
     
    

    /*constructor*/
    ParticleFilter(){}

    /*deconstructor*/
    ~ParticleFilter(){}
    
    /*initialize*/
    void init(double x,double y,double z);

    /*update weight according to observation*/
    void updateWeights(double x,double y,double z);
  
    /*predict the next timestamp*/
    void prediction();
    

    /* initialized Returns whether particle filter is initialized yet or not.*/
    const bool initialized() const {
        return is_gps_initialized && is_imu_initialized && is_lidar_initialized;
    }
   
    /*to calculate average result from all particles*/
    Eigen::Vector3d getAverage(const std::vector<Particle> &particles);
    

private:
    //numbers of particles
    int num_particles;
    std::vector<Particle> particles;
    //Flag 
    mutable std::shared_timed_mutex shMutex;
    bool is_gps_initialized = false;
    bool is_lidar_initialized = false;
    bool is_imu_initialized = false;
  

};

#endif