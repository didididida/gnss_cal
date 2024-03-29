#include <ros/ros.h>
#include <ros/console.h>
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
#include <shared_mutex>
#include "gnss_cal/gnssCal.h"
#include "gnss_cal/gnssToU.h"


#define GNSS_ELEVATION_THRESHOLD 15
std::shared_mutex iono_mu; //mutex for ionosphere
std::shared_mutex pos_mu;

std::unordered_map<int,double>record;

std::unordered_map<uint32_t,std::vector<gnss_comm::EphemBasePtr>>sat2ephem;//ephemris map
std::unordered_map<uint32_t,std::map<double,size_t>>sat2time_index;
std::vector<double>last_iono_param;
Eigen::Vector3d pos_lla;
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
    std::unique_lock lock(iono_mu);
    last_iono_param.clear();
    std::copy(iono_param.begin(),iono_param.end(),std::back_inserter(last_iono_param));
}

void reclla_cb(const sensor_msgs::NavSatFixConstPtr &recmsg){
     std::unique_lock lock(pos_mu);
     pos_lla = {recmsg->latitude,recmsg->longitude,recmsg->altitude};
}

void rangemeas_cb(const gnss_comm::GnssMeasMsgConstPtr &meas_msg){

    std::vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(meas_msg);
    gnss_cal::gnssCal msg;
    msg.header.stamp = ros::Time(gnss_comm::time2sec(gnss_meas[0]->time));
    msg.header.frame_id = "aft_mapped";
    Eigen::Vector3d pos_ecef;
    {
        std::shared_lock lock(pos_mu);
        pos_ecef = gnss_comm::geo2ecef(pos_lla);
    }

    // lla position
    Eigen::Vector3d rcv_lla = gnss_comm::ecef2geo(pos_ecef);
  
    for(auto obs:gnss_meas){
       
        //identify satellites type
        uint32_t sys = gnss_comm::satsys(obs->sat,NULL);
         
        //only process gps/bds/gal/glo
        if(sys!=SYS_GPS&&sys!=SYS_BDS&&sys!=SYS_GAL&&sys!=SYS_GLO)
        continue;
        //calculate the elevation, if less than 15 degree, discard
        //To count the number of satellites
        if(gnss_comm::satsys(obs->sat,NULL)==0)
        continue;
        if(obs->freqs.empty())continue;

        std::string sys_name ="";

        int freq_idx = -1;
        gnss_comm::L1_freq(obs,&freq_idx);
        
        // no L1, discard
        if(freq_idx<0)continue;
        //no ephemeris for this satellite continue
        if(!sat2time_index.count(obs->sat)){
            ROS_INFO("wait for %i-th satellite's ephemris data....",obs->sat);
            continue;
        }
        //no ionosphere parameters, break
        if(last_iono_param.empty()){
            ROS_INFO("wait for ionosphere parameters");
            continue;
        }
        if(!pos_ecef.any()){
            ROS_INFO("wait for receiver's position");
            return;
        }

        /*calculte multipath erorr (Attention not NLOS here)
          according to a paper upload in the github 
          make true reciever can recieve more than one band   
        */
        double freq_2_min = 0.0;
        double freq_2_max = 0.0;
        //troposphere delay
        double trop_delay = 0.0;
        //ionosphere delay
        double iono_delay = 0.0;
        //Miltipath estimated Mp;
        double Mp = 0.0;

        if(sys==SYS_GPS){
            sys_name = "gps";
            freq_2_min = FREQ2;
            freq_2_max = FREQ2;
        }else if(sys==SYS_BDS){
            sys_name = "bds";
            freq_2_min = FREQ2_BDS;
            freq_2_max = FREQ2_BDS;
        }else if(sys ==SYS_GAL){
            sys_name = "gal";
            freq_2_min = FREQ5;
            freq_2_max = FREQ5;
        } else if (sys == SYS_GLO)
        {  sys_name = "glo";
           freq_2_min = FREQ2_GLO - 7 * DFRQ2_GLO;
           freq_2_max = FREQ2_GLO + 6 * DFRQ2_GLO;
        }
        

        // find the index for L2/BD2/E5
        int freq2_idx = -1;
        for(int i = 0;i<obs->freqs.size();i++){
             if (obs->freqs[i] >= freq_2_min && obs->freqs[i] <= freq_2_max)
            {
                if (freq2_idx != NULL)
                    freq2_idx = i;
            }
        }
      
        if(freq2_idx<0)
        {
            ROS_INFO("No frequence L2");

        }else{
        //carrier phase for diffrent frequency
        double CP_1 = obs->cp[freq_idx]*(1.0/obs->freqs[freq_idx]);
        double CP_2 = obs->cp[freq2_idx]*(1.0/obs->freqs[freq2_idx]);
        CP_1 = CP_1*LIGHT_SPEED;
        CP_2 = CP_2*LIGHT_SPEED;
        double a =pow(obs->freqs[freq2_idx],2)/(pow(obs->freqs[freq_idx],2)-pow(obs->freqs[freq2_idx],2));
        //double b = (pow(obs->freqs[freq_idx],2)+pow(obs->freqs[freq2_idx],2))/(pow(obs->freqs[freq_idx],2)-pow(obs->freqs[freq2_idx],2));
        //Mp here includes multipath error and noise
        if(CP_1!=0&&CP_2!=0)
        Mp = obs->psr[freq_idx]-CP_1 + 2* a*(CP_1-CP_2);
        
        /*
        if(obs->sat == 46) Mp=0;
        if(obs->sat == 27) Mp -= 3671.971736/208;
        if(obs->sat == 39) Mp -= 1725.940494/200;
        if(obs->sat == 10) Mp -= 489.966076/139;
        if(obs->sat == 8)  Mp -= 162.195664/155;
        */

        //dual-frequency to calculate ionosphere delay
        iono_delay = a*(obs->psr[freq_idx]-obs->psr[freq2_idx]);
        }
        
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
            //std::cout<<"ephemris not valid"<<std::endl;
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
        
       
        double azel[2] = {0, M_PI/2.0};
        gnss_comm::sat_azel(pos_ecef,sat_ecef,azel);
        //if elevation angle is small than threshold, discard
        if(azel[1]<GNSS_ELEVATION_THRESHOLD*M_PI/180.0){
           
            continue;
        }
        
        
        //calculate troposphere delay
        trop_delay = gnss_comm::calculate_trop_delay(obs->time,rcv_lla,azel);
        
        //if no dual-frequency, use klobuchar model to estimate iono_delay
        if(freq2_idx<0||iono_delay==0)
        {
            std::shared_lock lock(iono_mu);
            iono_delay = gnss_comm::calculate_ion_delay(obs->time,last_iono_param,rcv_lla,azel);
        }

        //calculated pseudorange, eliminate the error of iono, trop, group delay 
        //and satellite clock drift.
        //receiver clock offset not included here
        double psr = obs->psr[freq_idx]- trop_delay - iono_delay+svdt*LIGHT_SPEED-tgd*LIGHT_SPEED;    
        //eliminate the estimated mulipath error
        gnss_cal::gnssToU submsg;
        submsg.CN0 = obs->CN0;
        submsg.sys = sys_name;
        submsg.psr = psr;
        submsg.sat = obs->sat;
        submsg.mp = Mp;
        submsg.ecefX = sat_ecef[0];
        submsg.ecefY = sat_ecef[1];
        submsg.ecefZ = sat_ecef[2];
        msg.meas.push_back(submsg);
    }
    
    msg.latitude = rcv_lla[0];
    msg.longitude =rcv_lla[1];
    msg.altitude = rcv_lla[2];
    pub.publish(msg);
}


int main(int argc,char*argv[]){


    ros::init(argc,argv,"gnss_cal");
    ros::NodeHandle nh;

    //subscribe message published by ublox_driver
    //This message includes ephemris
    ros::Subscriber ephem_sub=nh.subscribe<gnss_comm::GnssEphemMsg>("/ublox_driver/ephem",100,ephem_cb);
    ros::Subscriber gloephem_sub=nh.subscribe<gnss_comm::GnssGloEphemMsg>("/ublox_driver/glo_ephem",100,gloephem_cb);
    ros::Subscriber iono_param_sub=nh.subscribe<gnss_comm::StampedFloat64Array>("/ublox_driver/iono_params",100,ionoparam_cb);
    ros::Subscriber receiver_lla_sub=nh.subscribe<sensor_msgs::NavSatFix>("/ublox_driver/receiver_lla",100,reclla_cb);
    
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
