#include <ros/ros.h>
#include <sstream>
#include <math.h>
#include "gnss_comm/GnssEphemMsg.h"
#include "gnss_comm/gnss_constant.hpp"
#include "gnss_comm/gnss_ros.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <Eigen/Dense>
#include <unordered_map>
#include <thread>
#include <ros/callback_queue.h>
#include <mutex>
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"


std::mutex iono_mu; //mutex for ionosphere
std::mutex pos_mu;

std::unordered_map<uint32_t,std::vector<gnss_comm::EphemBasePtr>>sat2ephem;//ephemris map
std::unordered_map<uint32_t,std::map<double,size_t>>sat2time_index;
std::vector<double>last_iono_param;
Eigen::Vector3d pos_ecef;
//used in particle filter
std::unordered_map<int,std::pair<Eigen::Vector3d,double>>ref_map;

ros::Publisher pub;

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
void inputIonoParam(double ts,std::vector<double>&iono_param);
void ionoparam_cb(const gnss_comm::StampedFloat64ArrayConstPtr &ionoparam_msg){
    double ts = ionoparam_msg->header.stamp.toSec();
    std::vector<double>iono_params;
    std::copy(ionoparam_msg->data.begin(),ionoparam_msg->data.end(),std::back_inserter(iono_params));
    assert(iono_params.size()==8);
    inputIonoParam(ts,iono_params);
}

//update the newest iono parameters
void inputIonoParam(double ts,std::vector<double>&iono_param){
  
    if(iono_param.size()!=8)return;
    iono_mu.lock();
    last_iono_param.clear();
    std::copy(iono_param.begin(),iono_param.end(),std::back_inserter(last_iono_param));
    iono_mu.unlock();
}

void reclla_cb(const sensor_msgs::NavSatFixConstPtr &recmsg){
     pos_mu.lock();
     pos_ecef = {recmsg->latitude,recmsg->longitude,recmsg->altitude};
     pos_mu.unlock();
}

void rangemeas_cb(const gnss_comm::GnssMeasMsgConstPtr &meas_msg){

    std::vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(meas_msg);
    gnss_cal::gnssCal msg;
    pos_mu.lock();
    Eigen::Vector3d pos = pos_ecef;
    pos_mu.unlock();

    
    for(auto obs:gnss_meas){
       
        //identify satellites type
        uint32_t sys = gnss_comm::satsys(obs->sat,NULL);

        //only process gps/bds/gal/glo
        if(sys!=SYS_GPS&&sys!=SYS_BDS&&sys!=SYS_GAL&&sys!=SYS_GLO)
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

        if(!sat2time_index.count(obs->sat)){
            ROS_WARN("wait for %i-th satellite's ephemris data....",obs->sat);
            continue;
        }
        if(last_iono_param.empty()){
            ROS_WARN("wait for ionosphere parameters");
            return;
        }
        if(!pos.any()){
            ROS_WARN("wait for receiver's position");
            return;
        }

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
        double svdt = 0.0;
        double tgd = 0.0;
        if(sys==SYS_GLO){
            svdt = gnss_comm::geph2svdt(obs->time,std::dynamic_pointer_cast<gnss_comm::GloEphem>(best_ephem));
            sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<gnss_comm::GloEphem>(best_ephem), NULL);
            
        }else{
            svdt = gnss_comm::eph2svdt(obs->time,std::dynamic_pointer_cast<gnss_comm::Ephem> (best_ephem));
            sat_ecef = eph2pos(obs->time,std::dynamic_pointer_cast<gnss_comm::Ephem> (best_ephem), NULL);
            tgd = (std::dynamic_pointer_cast<gnss_comm::Ephem>(best_ephem))->tgd[0];
        }
        
        //calculate the ionosphere delay and troposphere delay
        double iono_delay = 0;
        double trop_delay = 0;
       
        double azel[2] = {0, M_PI/2.0};
        Eigen::Vector3d rcv_lla = gnss_comm::ecef2geo(pos);
        gnss_comm::sat_azel(pos,sat_ecef,azel);
        trop_delay = gnss_comm::calculate_trop_delay(obs->time,rcv_lla,azel);

        iono_mu.lock();
        iono_delay = gnss_comm::calculate_ion_delay(obs->time,last_iono_param,rcv_lla,azel);
        iono_mu.unlock();

        //calculated pseudorange, eliminate the error of iono, trop, group delay 
        //and satellite clock drift.
        double psr = obs->psr[freq_idx]-trop_delay-iono_delay+svdt*LIGHT_SPEED-tgd*LIGHT_SPEED;    
        gnss_cal::gnssToU submsg;
        submsg.CN0 = obs->CN0;
        submsg.psr = psr;
        submsg.sat = obs->sat;
        submsg.latitude = pos[0];
        submsg.longitude = pos[1];
        submsg.altitude = pos[2];
        submsg.ecefX = sat_ecef[0];
        submsg.ecefY = sat_ecef[1];
        submsg.ecefZ = sat_ecef[2];
        msg.meas.push_back(submsg);
    }
    pub.publish(msg);
}


int main(int argc,char*argv[]){


    ros::init(argc,argv,"gnss_cal");
    ros::NodeHandle nh;

    //subscribe message published by ublox_driver
    //This message includes ephemris
    ros::Subscriber ephem_sub=nh.subscribe<gnss_comm::GnssEphemMsg>("/ublox_driver/ephem",1,ephem_cb);
    ros::Subscriber gloephem_sub=nh.subscribe<gnss_comm::GnssGloEphemMsg>("/ublox_driver/glo_ephem",1,gloephem_cb);
    ros::Subscriber iono_param_sub=nh.subscribe<gnss_comm::StampedFloat64Array>("/ublox_driver/iono_params",1,ionoparam_cb);
    ros::Subscriber receiver_lla_sub=nh.subscribe<sensor_msgs::NavSatFix>("/ublox_driver/receiver_lla",1,reclla_cb);
    
    //create another thread for gnss meassurement callback function
    ros::NodeHandle nh_second;
    pub = nh_second.advertise<gnss_cal::gnssCal>("gnss_post_cal",1);
    ros::CallbackQueue rangemeas_cb_queue;
    nh_second.setCallbackQueue(&rangemeas_cb_queue);

    ros::Subscriber range_meas_sub=nh_second.subscribe<gnss_comm::GnssMeasMsg>("/ublox_driver/range_meas",1,rangemeas_cb);
    std::thread spinner_thread_a([&rangemeas_cb_queue](){
        ros::SingleThreadedSpinner spinner_a;
        spinner_a.spin(&rangemeas_cb_queue);
    });
   
    ros::spin();
    spinner_thread_a.join();

    return 0;
}
