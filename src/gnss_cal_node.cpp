#include <ros/ros.h>
#include <sstream>
#include <math.h>
#include "gnss_comm/GnssEphemMsg.h"
#include "gnss_comm/gnss_constant.hpp"
#include "gnss_comm/gnss_ros.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

using namespace message_filters;
std::unordered_map<int,std::vector<gnss_comm::EphemBasePtr>>sat2ephem;
std::unordered_map<int,std::map<double,size_t>>sat2time_index;
std::vector<double>last_iono_param;

//used in particle filter
std::unordered_map<int,std::pair<Eigen::Vector3d,double>>ref_map;
void inputEphem(gnss_comm::EphemBasePtr ephem_ptr){
    //tansform into second
    double toe = gnss_comm::time2sec(ephem_ptr->toe);
    //if there is a new ephemris
    if(sat2time_index.count(ephem_ptr->sat)==0||sat2time_index.at(ephem_ptr->sat).count(toe)==0){
        sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
        // the newest index of ephemris
        sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size()-1);
    }
}

//process ephemeris of BDS GPS GAL
void ephem_cb(const gnss_comm::GnssEphemMsgConstPtr &ephem_msg){
     gnss_comm::EphemPtr ephem = gnss_comm::msg2ephem(ephem_msg);
     inputEphem(ephem);
}


//Process ephemris of GLO
void gloephem_cb(const gnss_comm::GnssGloEphemMsgConstPtr &gloephem_msg){
     gnss_comm::GloEphemPtr gloephem = gnss_comm::msg2glo_ephem(gloephem_msg);
     inputEphem(gloephem);
}

//receive iono parameters
void ionoparam_cb(gnss_comm::StampedFloat64ArrayConstPtr &ionoparam_msg){
    double ts = ionoparam_msg->header.stamp.toSec();
    std::vector<double>iono_params;
    std::copy(ionoparam_msg->data.begin(),ionoparam_msg->data.end(),std::back_inserter(iono_params));
    assert(iono_params.size()==8);
    inputIonoParam(ts,iono_params);
}

//update the newest iono parameters
void inputIonoParam(double ts,std::vector<double>iono_param){
    if(iono_param.size()!=8)return;
    last_iono_param.clear();
    std::copy(iono_param.begin(),iono_param.end(),std::back_inserter(last_iono_param));
}

void rangemeas_cb(const gnss_comm::GnssMeasMsgConstPtr &meas_msg){

    std::vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(meas_msg);
    
    
    for(auto obs:gnss_meas){
        
        //identify satellites type
        uint32_t sys = gnss_comm::satsys(obs->sat,NULL);

        //only process gps/bds/gal/glo
        if(sys!=SYS_GPS||sys!=SYS_BDS||sys!=SYS_GAL||sys!=SYS_GLO)
        continue;

        //To count the number of satellites
        if(gnss_comm::satsys(obs->sat,NULL)==0)
        continue;
        if(obs->freqs.empty())continue;

        //only use L1
        int freq_idx = -1;
        gnss_comm::L1_freq(obs,&freq_idx);
        if(freq_idx<0)continue;
        double obs_time = gnss_comm::time2sec(obs->time);
        std::map<double, size_t>time2index = sat2time_index.at(obs->sat);
        double ephem_time = EPH_VALID_SECONDS;
        size_t ephem_index = -1;

        for(auto it:time2index){
            if(abs(obs_time-it.first)<ephem_time){
                ephem_time = abs(obs_time-it.first);
                ephem_index = it.second;
            }
        }

        if(ephem_time>=EPH_VALID_SECONDS){
            std::cout<<"ephemris not valid"<<std::endl;
        }

        const gnss_comm::EphemBasePtr &best_ephem=sat2ephem.at(obs->sat).at(ephem_index);
        Eigen::Vector3d sat_ecef;

        if(sys==SYS_GLO){
            sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<gnss_comm::GloEphem>(best_ephem), NULL);
        }else{
            sat_ecef = eph2pos(obs->time, std::dynamic_pointer_cast<gnss_comm::Ephem>(best_ephem), NULL); 
        }
        double psr = obs->psr.at(0);
    }
    
}


int main(int argc,char*argv[]){
    ros::init(argc,argv,"gnss_cal");
    ros::NodeHandle nh;
    //subscribe message published by ublox_driver
    //This message includes ephemris
    ros::Subscriber ephem_sub=nh.subscribe<gnss_comm::GnssEphemMsg>("/ublox_driver/ephem",1,ephem_cb);
    ros::Subscriber gloephem_sub=nh.subscribe<gnss_comm::GnssGloEphemMsg>("/ublox_driver/glo_ephem",1,gloephem_cb);
    ros::Subscriber iono_param_sub=nh.subscribe<gnss_comm::StampedFloat64Array>("/ublox_driver/iono_params",1,ionoparam_cb);
    ros::Subscriber range_meas_sub=nh.subscribe<gnss_comm::GnssMeasMsg>("/ublox_driver/range_meas",1,rangemeas_cb);
    
    message_filters::Subscriber<sensor_msgs::NavSatFix>receiver_lla_sub(nh,"/ublox_driver/receiver_lla",1);
  
    ros::spin();
    return 0;
}