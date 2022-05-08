/*particle filter class*/

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>
#include "helper_function.h"
#include <sensor_msgs/NavSatFix.h>
#include <unordered_map>
#include <random>
#include <Eigen/Dense>
#include "satellite_subscriber.h"
#include "plane_subscriber.h"

using namespace lidar_localization;

class Particle{
public:
    Particle(){};
    int id;
    Eigen::Vector3d p_ecef;
    double weight;
};


class ParticleFilter{
public:
     
    /*constructor*/
    ParticleFilter(const Eigen::Matrix<float,4,1>&q,const Eigen::Vector3d &pos_lla);
    /*deconstructor*/
    ~ParticleFilter(){}
    
    /*initialize particles*/
    void init_pf();

    /*update weight according to observation*/
    Eigen::Vector3d updateWeights( const ALL_plane &planes, const ALL_sat &sat);

    /*scatter particles around gps pos*/
    void scatter(const Eigen::Vector3d& pos_lla);
    
    /* initialized Returns whether particle filter is initialized yet or not.*/
    const bool initialized() const {
        return is_initialized;
    }
   
    /*to calculate average result from all particles*/
    Eigen::Vector3d getAverage(const std::vector<Particle> &particles);
   
private:
    //numbers of particles
    int num_particles;
    std::vector<Particle> particles;
    Eigen::Vector3d lla;
    Eigen::Vector3d ecef;
    bool is_initialized =false;

    Eigen::Matrix<float,3,3> C_N2B;
    // variation of pseudorange's error
    double SIGMA_P = 100;

    //quoternion
    Eigen::Matrix<float,4,1> q;
   
    std::vector<double>grid={-6.0,-4.0,-2.0,0.0,2.0,4.0,6.0};
    std::vector<Eigen::Vector3d> enu_p;
    
    
};

#endif