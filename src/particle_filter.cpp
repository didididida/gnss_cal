#include "particle_filter.h"

void ParticleFilter::init(){
    particles.resize(num_particles);
    is_initialized = true;
}

//scatter particles around GPS position
void ParticleFilter::scatter(){
int count = 0;
for(int i=0;i<7;i++){
    for(int j=0;j<7;j++){
    Particle p;
    p.x = utm_original_point.easting+grid[i];
    p.y = utm_original_point.northing+grid[j];
    p.z=utm_original_point.altitude;
    p.weight = 0;
    particles[count]=p;
    count++;
    }
 }
}


//translate from wgs to utm
void ParticleFilter::WGS2UTM(const double &lat,const double &lon,const double &alt){
    geographic_msgs::GeoPointStampedPtr geo_msg(new geographic_msgs::GeoPointStamped);
    geo_msg->position.latitude = lat;
    geo_msg->position.longitude =lon;
    geo_msg->position.altitude = alt;
    geodesy::fromMsg(geo_msg->position,utm_original_point);
}

//translate from utm to wgs
void UTM2WGS(Eigen::Vector3d &geo, geodesy::UTMPoint utm_point){
    geographic_msgs::GeoPoint geo_msg;
    geo_msg = geodesy::toMsg(utm_point);
    geo[0]=geo_msg.latitude;
    geo[1]=geo_msg.longitude;
    geo[2]=geo_msg. altitude;
}

//core function, calculate weight about particles
void ParticleFilter::updateWeights(double lat,double lon,double alt, Eigen::Matrix<float,4,1> q,std::vector<Plane>planes
    , std::vector<Sat_info>sat){
    if(!initialized()){
        init();
    }
    //transformation matrix from end to body frame
    C_N2B = quat2dcm(q);

    //translate wsg into utm 
    WGS2UTM(lat,lon,alt);

    //scatter around utm position
    scatter();


    /*for each particle do ray-tracing process*/
  for (const auto &p :particles){
    //from utm to wgs, geo_p is particle's position in wgs;
    Eigen::Vector3d geo_p;
    geodesy::UTMPoint utm_p(p.x,p.y,utm_original_point.altitude,utm_original_point.zone,utm_original_point.band);
    UTM2WGS(geo_p,utm_p);
    
    //in ecef coordinate system
    Eigen::Vector3d ecef_p;
    ecef_p = lla2ecef(geo_p);
    
    //position for lidar in lidar's own system
    Point p_lidar(0,0,0);
    //hashmap to store estimated psudorange
    std::unordered_map<int,double> id_psr;

            /*For each satellite do ray-tracing*/
            for(int i=0;i<sat.size();i++){


            // estimated pseudorange
            double psr_estimated = DBL_MAX;
            double psr_mea = sat[i].psr;

            Eigen::Vector3d sat_ecef;
            sat_ecef[0]=sat[i].ecefX;
            sat_ecef[1]=sat[i].ecefY;
            sat_ecef[2]=sat[i].ecefZ;
            Eigen::Vector3d sat_end;
            sat_end = ecef2ned(sat_ecef,ecef_p);
            
            //position of satellite in body frame 
            Eigen::Vector3d sat_pos;
            sat_pos = C_N2B*sat_end;


            /*For each plane detected by lidar*/
            for(int j=0;j<planes.size();j++){

               //do mirror about this planes
               Point p_mir;
               p_mir = mirror(p_lidar,planes[i]);
               Point p_sat (sat_pos[0],sat_pos[1],sat_pos[2]);
               Point p_intersect = linePlaneIntersection(p_mir,p_sat,planes[j]);
            
               //validate intersec point within this plane
               if(p_intersect._z>planes[j].z_max||p_intersect._z<planes[j].z_min)
               {
                  double tmp_psr = point2point(p_lidar,p_sat);
                  psr_estimated = std::min(psr_estimated,tmp_psr);
               }
               else
               {
                  double tmp_psr = point2point(p_mir,p_sat);
                  psr_estimated = std::min(psr_estimated,tmp_psr);
               }
            }
        
            double error = abs(psr_estimated-psr_mea);
            id_psr[sat[i].id]=error;
        }
        
        
    }
}

void ParticleFilter::updateWeights(){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    updateWeights(lat,lon,alt,q,planes,sat);
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
    is_sat_update = true;
    ROS_INFO("GPS LOCATION UPDATE");
}

//update position info from gps
void ParticleFilter::updateGps(const sensor_msgs::NavSatFixConstPtr &pos_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    lat = pos_msg->latitude;
    lon = pos_msg->longitude;
    alt = pos_msg->altitude;
    is_gps_update = true;
    if(is_gps_update&&is_imu_update&&is_lidar_update&&is_sat_update){
        updateWeights();
    }else{
        return;
    }
}

//From imu's quaternion to get matrix from enu to body frame
void ParticleFilter::updateImu(const sensor_msgs::ImuConstPtr &imu_msg){
    std::unique_lock<std::shared_timed_mutex> lock(shMutex);
    q(0,0) = imu_msg->orientation.w;
    q(1,0) = imu_msg->orientation.x;
    q(2,0) = imu_msg->orientation.y;
    q(3,0) = imu_msg->orientation.z;
    is_imu_update = true;
}