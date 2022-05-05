#include "particle_filter.h"


void ParticleFilter::init_ros(){
    //_sub_gps = _nh.subscribe("/ublox/gps",1,&ParticleFilter::updateGps,this);
    _sub_imu = _nh.subscribe("/imu/data",1,&ParticleFilter::updateImu,this);
    _sub_lidar = _nh.subscribe("/planecheck/planes_coefficient",1,&ParticleFilter::updateLidar,this);
    _sub_sat = _nh.subscribe("/gnss_post_cal",1,&ParticleFilter::updateSat,this);
    _pub_nav = _nh.advertise<sensor_msgs::NavSatFix>("post_nav",1);

    ROS_INFO("ROSNODE FOR PARTICLE FILTER INITIALIZED");
}

// in enu coordinate system, scatter particles in 3D
void ParticleFilter::init_pf(){
    //particles.resize(num_particles);
    for(int i=0;i<7;i++){
        for(int j=0;j<7;j++){
            enu_p.push_back({grid[i],grid[j],0});
            enu_p.push_back({grid[i],grid[j],-2});
            enu_p.push_back({grid[i],grid[j],2});
        }
    }
    is_initialized = true;
}

//scatter particles around position
//particles in enu already know.
void ParticleFilter::scatter(const Eigen::Vector3d& pos_lla){
std::vector<Eigen::Vector3d> particles_ecef;
for(int i=0;i<enu_p.size();i++){
    Eigen::Vector3d p_ecef;
    p_ecef = enu2ecef(enu_p[i],pos_lla);
    Particle p;
    p.id=i;
    p.p_ecef=p_ecef;
    p.weight=1;
    particles.push_back(p);
  }
}


//core function, calculate weight about particles
void ParticleFilter::updateWeights(double lat,double lon,double alt, Eigen::Matrix<float,4,1> q,std::vector<Plane>planes
    , std::vector<Sat_info>sat){
    if(!initialized()){
        init_pf();
    }
    //transformation MATRIX from end to body frame
    C_N2B = quat2dcm(q);

    Eigen::Vector3d pos_ecef;
    Eigen::Vector3d pos_lla;
    //translate from lla to ecef for gps raw data
    pos_lla={lat,lon,alt};

    //scatter around enu's origin
    //return particles in ecef
    scatter(pos_lla);
     
    //hashmap to store estimated psudorange
    std::unordered_map<int,double> id_error;

    /*for each particle do ray-tracing process*/
    for (const auto &p :particles){

        //in ecef coordinate system
        Eigen::Vector3d ecef_p;
        Eigen::Vector3d lla_p;
        ecef_p = p.p_ecef;
        lla_p = gnss_comm::ecef2geo(ecef_p);
        //position for lidar in lidar's system
        Point p_lidar(0,0,0);

        //average error about psudorange
        double avr_error = 0.0;
        double sum_error = 0.0;

        //record the max elevation's satellite as reference statellite.
        double max_ele = 0.0;
        double ref_rcv_clock = 0;
        int ref_index = 0;
        //hashmap to store elevation for all satellites
        std::unordered_map<int,double>ele_all_sat;

        for(int i=0;i<sat.size();i++){
            double azel[2] = {0, M_PI/2.0};
            Eigen::Vector3d sat_ecef;
            sat_ecef(0)=sat[i].ecefX;
            sat_ecef(1)=sat[i].ecefY;
            sat_ecef(2)=sat[i].ecefZ;
            gnss_comm::sat_azel(ecef_p,sat_ecef,azel);
            ele_all_sat[sat[i].id] = azel[1];
            if(azel[1]>max_ele){
                max_ele = azel[1];
                Eigen::Vector3d tmp;
                tmp = sat_ecef - ecef_p;
                ref_rcv_clock = sat[i].psr-sat[i].mp - tmp.norm();
                ref_index = i;
            }
        }
        
        /*For each satellite do ray-tracing*/
        for(int i=0;i<sat.size();i++){
            
            //reference satellite's error is neglected
            if(i==ref_index)continue;
             
            // estimated pseudorange
            double psr_estimated = DBL_MAX;
            // measured pseudorange
            double psr_mea = sat[i].psr;

            Eigen::Vector3d sat_ecef;
            sat_ecef(0)=sat[i].ecefX;
            sat_ecef(1)=sat[i].ecefY;
            sat_ecef(2)=sat[i].ecefZ;

            Eigen::Vector3d sat_enu;
            sat_enu = gnss_comm::ecef2enu(sat_ecef,lla_p);

            //position of satellite in body frame 
            Eigen::Vector3d sat_local;
            Eigen::Matrix<double,3,3>R=C_N2B.cast<double>();
            sat_local = R*sat_enu;
        
                Point p_sat (sat_local[0],sat_local[1],sat_local[2]);

                /*-----------------------------------------
                 validate intersec point within this plane
                 calculate the estimated pesudorange
                 this part need to be improved
                -----------------------------------------*/

                // if this plane is blocked or not
                bool is_blocked = false;
                std::set<int>block_index;

                for(int j=0;j<planes.size();j++){
                //intersect between lidar and sat
                     if(isIntersect(p_lidar,p_sat,planes[j])){
                     is_blocked = true;
                     block_index.insert(j);
                     }
                }
            
                //Calculate The NLOS Psudorange
                // If satellite is not blocked by all planes

                if(!is_blocked){
                 psr_estimated = point2point(p_lidar,p_sat)+ref_rcv_clock;

                }else{
                
                    for(int j=0;j<planes.size();j++){
                    
                    //block by this plane skip it
                    if(block_index.count(j)){
                    continue;
                    }
                   
                    double lidar2wall = point2planedistance(p_lidar,planes[j]);
                    double ele = ele_all_sat[sat[i].id];

                    if(ele==M_PI/2.0){
                      psr_estimated = 0;
                      continue;
                    }

                    double gama1 = lidar2wall/sin(M_PI/2.0-ele);
                    double gama2 = gama1 * cos(2*ele);
                    double delta_psd = gama1 + gama2;

                       if(delta_psd<psr_estimated){
                        Eigen::Vector3d d;
                        d = ecef_p - sat_ecef;
                        psr_estimated = delta_psd+d.norm()+ref_rcv_clock;}
                    }   
                }

            double error = abs(psr_estimated-psr_mea);
            sum_error += error;
           
        }
        //avr_error = sum_error/sat.size();
        id_error[p.id]=sum_error;
    }

        //assign weight for all particles based on its error.
        //Gaussain distrbution and maen believed to be zero, variation is 0.5
        double sum_weight = 0.0;
        for(auto &p:particles){
        p.weight = exp(-pow(id_error[p.id],2)/SIGMA_P);
        sum_weight += p.weight;
        }

        //calculate average position data
        Eigen::Vector3d avr_pos;
        avr_pos.setZero();
        for(const auto &p :particles){
        avr_pos.x() += p.p_ecef.x() * (p.weight/sum_weight);
        avr_pos.y() += p.p_ecef.y() * (p.weight/sum_weight);
        avr_pos.z() += p.p_ecef.z() * (p.weight/sum_weight);
        }
    
   
    Eigen::Vector3d result;
    result = gnss_comm::ecef2geo(avr_pos);
    //publish post navigation gps
    sensor_msgs::NavSatFix post_nav_msg;
    post_nav_msg.header = restore_.header;
    post_nav_msg.status = restore_.status;
    post_nav_msg.latitude = result[0];
    post_nav_msg.longitude = result[1];
    post_nav_msg.altitude = result[2];
    _pub_nav.publish(post_nav_msg);
}

void ParticleFilter::updateWeights(){
    std::shared_lock lock(shMutex);
    updateWeights(lat,lon,alt,q,planes,sat);
}

//update the Plane infomation
void ParticleFilter::updateLidar(const gnss_cal::detect_planesConstPtr &plane_msg){
    std::unique_lock lock(shMutex);
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
    std::unique_lock lock(shMutex);
    //clear the previous info about satellite's position
    sat.clear();
    for(auto &it:gps_msg->meas){
        Sat_info s;
        s.id = it.sat;
        s.CN0 = it.CN0.at(0);
        s.ecefX = it.ecefX;
        s.ecefY =it.ecefY;
        s.ecefZ = it.ecefZ;
        s.psr = it.psr;
        sat.push_back(s);
    }
    lat = gps_msg->latitude;
    lon = gps_msg->longitude;
    alt = gps_msg->altitude;
    is_sat_update = true;
    ROS_INFO("satellite and raw location update");
    if(is_gps_update&&is_imu_update&&is_lidar_update&&is_sat_update){
        updateWeights();
    }else{
        return;
    }
}

//update position info from gps
void ParticleFilter::updateGps(const sensor_msgs::NavSatFixConstPtr &pos_msg){
    std::unique_lock lock(shMutex);
    restore_.header=pos_msg->header;
    restore_.status=pos_msg->status;
    is_gps_update = true;
}

//From imu's quaternion to get matrix from enu to body frame
void ParticleFilter::updateImu(const sensor_msgs::ImuConstPtr &imu_msg){
    std::unique_lock lock(shMutex);
    q(0,0) = imu_msg->orientation.w;
    q(1,0) = imu_msg->orientation.x;
    q(2,0) = imu_msg->orientation.y;
    q(3,0) = imu_msg->orientation.z;
    is_imu_update = true;
    ROS_INFO("IMU update");
}


int main(int argc,char*argv[]){
    ros::init(argc,argv,"particle_filter");
    ros::NodeHandle nh("~");
    ParticleFilter pf (nh);
    pf.spin();
    return 0;
}