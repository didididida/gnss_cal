#include "particle_filter.h"


Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}
// major eccentricity squared
constexpr double ECC2 = 0.0066943799901;
// earth semi-major axis radius (m)
constexpr double EARTH_RADIUS = 6378137.0;

  constexpr std::pair<double, double> earthradius(double lat) {
  double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
  double Rew = EARTH_RADIUS / sqrt(denom);
  double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
  return (std::make_pair(Rew, Rns));
}

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
 Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew, denom;
  Eigen::Matrix<double,3,1> ecef;
  std::tie(Rew, std::ignore) = earthradius(lla(0,0));
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
  Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  return ned;
  }
  

  Eigen::Vector3d enu2ecef(Eigen::Vector3d enu,Eigen::Vector3d lla){
     Eigen::Matrix<double,3,3>R;
     R(0,0)=-sin(lla[1]);
     R(0,1)= cos(lla[1]);
     R(0,2)= 0;
     R(1,0)= -sin(lla[0])*cos(lla[1]);
     R(1,1) = -sin(lla[0])*sin(lla[1]);
     R(1,2) = cos(lla[1]);
     R(2,0) = cos(lla[0])*cos(lla[1]);
     R(2,1) = cos(lla[0])*sin(lla[1]);
     R(2,2) = sin(lla[0]);
     Eigen::Vector3d ecef;
     ecef = R.transpose()*enu;
     return ecef;
}

/*get the intersection point between A line and a plane*/
 Point linePlaneIntersection(const Point&p1,const Point&p2,const Plane& plane){
      Point pointIntersection;
      double vpt = plane.normal[0]*(p2._x-p1._x)+plane.normal[1]*(p2._y-p1._y)
      +plane.normal[2]*(p2._z-p1._z);
      if(vpt==0){
            ROS_INFO("LINE IS PARALLEL TO THIS PLANE");
            pointIntersection._x = DBL_MAX;
            pointIntersection._y = DBL_MAX;
            pointIntersection._z = DBL_MAX;
      }else{
           double t = ((plane.p._x-p1._x)*plane.normal[0]
           +(plane.p._y-p1._y)*plane.normal[1]+(plane.p._z-p1._z)*plane.normal[2])/vpt;
           pointIntersection._x=p1._x+(p2._x-p1._x)*t;
           pointIntersection._y=p1._y+(p2._y-p1._y)*t;
           pointIntersection._z=p1._z+(p2._z-p1._z)*t;
      }
      return pointIntersection;
}

 bool isIntersect(const Point&p1,const Point& p2,Plane& plane){
     Point p = linePlaneIntersection(p1,p2,plane);
     if(p._z<plane.z_min||p._z>plane.z_max)
     return false;
     else return true;
}

 bool isOnline(const Point&p1,const Point& p2, Point&p){

      if(p._x>std::min(p1._x,p2._x)&&p._x<std::max(p1._x,p2._x)
      &&p._y>std::min(p1._y,p2._y)&&p._y<std::max(p1._y,p2._y)
      &&p._z>std::min(p1._z,p2._z)&&p._z<std::max(p1._z,p2._z)){
        return true;
      }else{
        return false;
      }
}

 double point2planedistance(const Point&point,const Plane&plane){
       double f1=fabs(point._x*plane._A+point._y*plane._B+point._z*plane._C+plane._D);
       double f2=sqrt(pow(plane._A,2)+pow(plane._B,2)+pow(plane._C,2));
       return f1/f2;
}

 double point2point(const Point&p1,const Point&p2){
        double f1 = pow(p1._x-p2._x,2)+pow(p1._y-p2._y,2)+pow(p1._z-p2._z,2);
        double f2 = sqrt(f1);
        return f2;
}




ParticleFilter::ParticleFilter(const Eigen::Matrix<float,4,1>&q, const Eigen::Vector3d &pos_lla){
    C_N2B = quat2dcm(q);
    lat = pos_lla[0];
    lon = pos_lla[1];
    alt = pos_lla[2];
    init_pf();
    lla = {lat,lon,alt};
    scatter(lla);
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
Eigen::Vector3d ParticleFilter::updateWeights(const ALL_plane &planes, const ALL_sat &sat){
    if(!initialized()){
        init_pf();
    }

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

        for(int i=0;i<sat.sats.size();i++){
            double azel[2] = {0, M_PI/2.0};
            Eigen::Vector3d sat_ecef;
            sat_ecef(0)=sat.sats[i].ecefX;
            sat_ecef(1)=sat.sats[i].ecefY;
            sat_ecef(2)=sat.sats[i].ecefZ;
            gnss_comm::sat_azel(ecef_p,sat_ecef,azel);
            ele_all_sat[sat.sats[i].id] = azel[1];
            if(azel[1]>max_ele){
                max_ele = azel[1];
                Eigen::Vector3d tmp;
                tmp = sat_ecef - ecef_p;
                ref_rcv_clock = sat.sats[i].psr-sat.sats[i].mp - tmp.norm();
                ref_index = i;
            }
        }
        
        /*For each satellite do ray-tracing*/
        for(int i=0;i<sat.sats.size();i++){
            
            //reference satellite's error is neglected
            if(i==ref_index)continue;
             
            // estimated pseudorange
            double psr_estimated = DBL_MAX;
            // measured pseudorange
            double psr_mea = sat.sats[i].psr;

            Eigen::Vector3d sat_ecef;
            sat_ecef(0)=sat.sats[i].ecefX;
            sat_ecef(1)=sat.sats[i].ecefY;
            sat_ecef(2)=sat.sats[i].ecefZ;

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

                for(int j=0;j<planes.planes.size();j++){
                //intersect between lidar and sat
                     Plane p (
                            planes.planes[j].a,
                            planes.planes[j].b,
                            planes.planes[j].c,
                            planes.planes[j].d,
                            planes.planes[j].z_max,
                            planes.planes[j].z_min
                     );

                     if(isIntersect(p_lidar,p_sat,p)){
                     is_blocked = true;
                     block_index.insert(j);
                     }
                }
            
                //Calculate The NLOS Psudorange
                // If satellite is not blocked by all planes

                if(!is_blocked){
                 psr_estimated = point2point(p_lidar,p_sat)+ref_rcv_clock;

                }else{
                
                    for(int j=0;j<planes.planes.size();j++){
                    
                    //block by this plane skip it
                    if(block_index.count(j)){
                    continue;
                    }
                   
                    Plane p (
                            planes.planes[j].a,
                            planes.planes[j].b,
                            planes.planes[j].c,
                            planes.planes[j].d,
                            planes.planes[j].z_max,
                            planes.planes[j].z_min
                     );
                    
                    double lidar2wall = point2planedistance(p_lidar,p);
                    double ele = ele_all_sat[sat.sats[i].id];

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
    return {result[0],result[1],result[2]};
}



