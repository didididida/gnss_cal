/*particle filter class*/

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <vector>
#include "helper_function.h"
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"
#include "gnss_cal/detect_planes.h"
#include "gnss_cal/single_plane.h"


struct Particle{
    int id;
    double x;
    double y;
    double z;
    double weight;
};


class ParticleFilter{
public:
     
    std::vector<Particle> particles;

    /*constructor*/
    ParticleFilter():num_particles(0),is_initialized(false){}

    /*deconstructor*/
    ~ParticleFilter(){}
    
    /*initialize*/
    void init(double x,double y,double z);

    /*update weight according to observation*/
    void updateWeights();

    /*predict the next timestamp*/
    void prediction();
    

    /* initialized Returns whether particle filter is initialized yet or not.*/
    const bool initialized() const {
        return is_initialized;
    }

private:
    //numbers of particles
    int num_particles;

    //Flag 
    bool is_initialized;

    //weights of all particles
    std::vector<double>weights;
};

#endif