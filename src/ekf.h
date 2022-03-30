#pragma once

#include <stdint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>
#include <mutex>
#include <shared_mutex>

//acceleration due to gravity
constexpr float G = 9.807f;
// Correlation time or time constant
constexpr float TAU_A = 100.0f;
// Correlati1on time or time constant
constexpr float TAU_G = 50.0f;

// Initial set of covariance
constexpr float P_P_INIT = 10.0f;
constexpr float P_V_INIT = 1.0f;
constexpr float P_A_INIT = 0.34906f;
constexpr float P_HDG_INIT = 3.14159f;
constexpr float P_AB_INIT = 0.9810f;
constexpr float P_GB_INIT = 0.01745f;
// major eccentricity squared
constexpr double ECC2 = 0.0066943799901;
// earth semi-major axis radius (m)
constexpr double EARTH_RADIUS = 6378137.0;

struct gpsPosData{
    uint64_t pose_time;
    double lat;
    double lon;
    double alt;
};

using gpsPosDataPtr = std::shared_ptr<gpsPosData>;

struct gpsVelData{
   uint64_t vel_time;
   double vN; //north
   double vE; //east
   double vD; //Down
};

using gpsVelDataPtr = std::shared_ptr<gpsVelData>;

struct imuData{
   uint64_t imu_time;
   float gyroX; //gyro
   float gyroY;
   float gyroZ;
   float acclX; //accelerator
   float acclY;
   float acclZ;
};
using imuDataPtr = std::shared_ptr<imuData>;

struct magData {
    uint64_t mag_time;
    float hX;
    float hY;
    float hZ;
};
using magDataPtr = std::shared_ptr<magData>;

struct ekfState {
    uint64_t timestamp;
    Eigen::Vector3d lla;       // in radians
    // Velocities
    Eigen::Vector3d velNED;
    // The imu data.
    Eigen::Matrix<float,3,1> linear;
    Eigen::Matrix<float,3,1> angular;
    // Eular angles
    Eigen::Matrix<float,4,1> quat;       // Quaternion
    Eigen::Vector3d accl_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.
    // Covariance.
    Eigen::Matrix<float, 15, 15> cov;
};

/*class for the extened kalman filter*/
class ekfNav{
   public:
    // ekf_update
    void ekf_update( uint64_t time/*, unsigned long TOW*/,   /* Time, Time of the week from GPS */
                    double vn, double ve, double vd,    /* Velocity North, Velocity East, Velocity Down */
                    double lat, double lon, double alt, /* GPS latitude, GPS longitude, GPS/Barometer altitude */
                    float p, float q, float r,          /* Gyro P, Q and R  */
                    float ax, float ay, float az,       /* Accelarometer X, Y and Z */
                    float hx, float hy, float hz        /* Magnetometer HX, HY, HZ */ );
    
    //return if ekf is initialized
    bool initialized (){return initialized_;}
    // return pitch/roll  angle, rad
    float getPitch_rad() { return theta; }
    float getRoll_rad()         { return phi; }
     // returns the heading angle, rad
    float getHeadingConstrainAngle180_rad()      { return constrainAngle180(psi); }
    float getHeading_rad()      { return psi; }

     // returns the INS latitude, rad
    double getLatitude_rad()    { return lat_ins; }
    // returns the INS longitude, rad
    double getLongitude_rad()   { return lon_ins; }
    // returns the INS altitude, m
    double getAltitude_m()      { return alt_ins; }
    // returns the INS north velocity, m/s
    double getVelNorth_ms()     { return vn_ins; }
    // returns the INS east velocity, m/s
    double getVelEast_ms()      { return ve_ins; }
    // returns the INS down velocity, m/s
    double getVelDown_ms()      { return vd_ins; }

    // returns the INS ground track, rad
    float getGroundTrack_rad()  { return atan2f((float)ve_ins,(float)vn_ins); }
    // returns the gyro bias estimate in the x,y,z direction, rad/s
    float getGyroBiasX_rads(){return gbx;}
    float getGyroBiasY_rads(){return gby;}
    float getGyroBiasZ_rads(){return gbz;}
 
     // returns the accel bias estimate in the x,y,z direction, m/s/s
    float getAccelBiasX_mss()   { return abx; }
    float getAccelBiasY_mss()   { return aby; }
    float getAccelBiasZ_mss()   { return abz; }
    // return pitch, roll and yaw
    std::tuple<float,float,float> getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz);
    
    /*For ROS*/
    bool imuDataUpdateEKF(const imuDataPtr imu, ekfState* ekfOut);
    void magDataUpdateEKF(const magDataPtr mag);
    void gpsPosDataUpdateEKF(const gpsPosDataPtr pos);
    void gpsVelDataUpdateEKF(const gpsVelDataPtr vel);
    void setAcclNoise (float acclNoise) { SIG_W_A = acclNoise; }
    void setAcclBias  (float acclBias)  { SIG_A_D = acclBias;  }
    void setGyroNoise (float gyroNoise) { SIG_W_G = gyroNoise; }
    void setGyroBias  (float gyroBias)  { SIG_G_D = gyroBias;  }
    void setStdDevGpsPosNE(float stdDevGpsPosNE)    { SIG_GPS_P_NE = stdDevGpsPosNE; }
    void setStdDevGpsPosD(float stdDevGpsPosD)      { SIG_GPS_P_D  = stdDevGpsPosD;  }
    void setStdDevGpsVelNE(float stdDevGpsVelNE)    { SIG_GPS_V_NE = stdDevGpsVelNE; }
    void setStdDevGpsVelD(float stdDevGpsVelD)      { SIG_GPS_V_D  = stdDevGpsVelD;  }
    
    
   
   /*private member variables and functions*/
   private:
   float SIG_W_A = 0.05f; //acceleration
   float SIG_W_G = 0.00175f; //gyro
    // Std dev of Accelerometer Markov Bias
   float SIG_A_D = 0.01f;
   // Std dev of correlated gyro bias
   float SIG_G_D = 0.00025;
   // GPS measurement noise std dev (m)
   float SIG_GPS_P_NE = 3.0f;
   float SIG_GPS_P_D = 6.0f;
   // GPS measurement noise std dev (m/s)
   float SIG_GPS_V_NE = 0.5f;
   float SIG_GPS_V_D = 1.0f;

   gpsPosDataPtr pGpsPosDat;
   gpsVelDataPtr pGpsVelDat;
   imuDataPtr pImuDat;
   magDataPtr pMagDat;
   
   mutable std::shared_mutex shMutex;
   bool initialized_ = false;
   bool is_gps_pos_initialized = false;
   bool is_gps_vel_initialized = false;
   bool is_imu_initialized = false;
   bool is_mag_initialized = false;


   uint64_t _tprev; //timing
   unsigned long previousTOW;
   //estimated attitude (angle)
   float phi, theta, psi;
   //estimated NED velocity
   double vn_ins,ve_ins,vd_ins;
   //estimated location
   double lat_ins,lon_ins,alt_ins;
   // magnetic heading corrected for roll and pitch angle
   float Bxc,Byc;
   //accelerator bias for x,y,z
   float abx = 0.0,aby = 0.0,abz=0.0;
   //gyro bias for x,y,z
   float gbx = 0.0,gby = 0.0,gbz = 0.0;
   //earth radius at location
   double Re, Rn, denom;
   //state Matrix 15*15
   Eigen::Matrix<float,15,15>Fs = Eigen::Matrix<float,15,15>::Identity();
   //state transition matrix
   Eigen::Matrix<float,15,15>PHI = Eigen::Matrix<float,15,15>::Zero();

   //convariance Matrix
   Eigen::Matrix<float,15,15>P = Eigen::Matrix<float,15,15>::Zero();
   
   //process noise transfromation
   Eigen::Matrix<float,15,12>Gs = Eigen::Matrix<float,15,12>::Zero();

   Eigen::Matrix<float,12,12>Rw = Eigen::Matrix<float,12,12>::Zero();

   //process noise Matrix
   Eigen::Matrix<float,15,15>Q = Eigen::Matrix<float,15,15>::Zero();
   //gravity model
   Eigen::Matrix<float,3,1> grav = Eigen::Matrix<float,3,1>::Zero();
   //rotation rate
   Eigen::Matrix<float,3,1> om_ib = Eigen::Matrix<float,3,1>::Zero();
   
   //specific force
   Eigen::Matrix<float,3,1> f_b = Eigen::Matrix<float,3,1>::Zero();
   //DCM direction cosine matrix
   Eigen::Matrix<float,3,3> C_N2B = Eigen::Matrix<float,3,3>::Zero();
   //DCM transpose
   Eigen::Matrix<float,3,3> C_B2N = Eigen::Matrix<float,3,3>::Zero();
   // to get dxdt
   Eigen::Matrix<float,3,1> dx = Eigen::Matrix<float,3,1>::Zero();
   Eigen::Matrix<double,3,1> dxd = Eigen::Matrix<double,3,1>::Zero();

   //estimated INS
   Eigen::Matrix<double,3,1>estimated_ins = Eigen::Matrix<double,3,1>::Zero();
   //NED velocity INS
   Eigen::Matrix<double,3,1>V_ins = Eigen::Matrix<double,3,1>::Zero();
    // LLA INS
   Eigen::Matrix<double,3,1> lla_ins = Eigen::Matrix<double,3,1>::Zero();
   // NED velocity GPS
   Eigen::Matrix<double,3,1> V_gps = Eigen::Matrix<double,3,1>::Zero();
    // LLA GPS
    Eigen::Matrix<double,3,1> lla_gps = Eigen::Matrix<double,3,1>::Zero();    
   // position ecef ins
    Eigen::Matrix<double,3,1> pos_ecef_ins = Eigen::Matrix<double,3,1>::Zero(); 
   //position ned ins
    Eigen::Matrix<double,3,1> pos_ned_ins = Eigen::Matrix<double,3,1>::Zero(); 
   //position ecef gps
    Eigen::Matrix<double,3,1> pos_ecef_gps = Eigen::Matrix<double,3,1>::Zero(); 
    //position ned gps
    Eigen::Matrix<double,3,1> pos_ned_gps = Eigen::Matrix<double,3,1>::Zero(); 
    //quat
    Eigen::Matrix<float,4,1> quat = Eigen::Matrix<float,4,1>::Zero();
    //dquat
    Eigen::Matrix<float,4,1> dq = Eigen::Matrix<float,4,1>::Zero();
    // difference between GPS and INS
    Eigen::Matrix<float,6,1> y = Eigen::Matrix<float,6,1>::Zero();
    // GPS measurement noise
    Eigen::Matrix<float,6,6> R = Eigen::Matrix<float,6,6>::Zero();
    Eigen::Matrix<float,15,1> x = Eigen::Matrix<float,15,1>::Zero();
    // Kalman Gain
    Eigen::Matrix<float,15,6> K = Eigen::Matrix<float,15,6>::Zero();
    Eigen::Matrix<float,6,15> H = Eigen::Matrix<float,6,15>::Zero();
    // skew symmetric
    Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w);

    /*below is private memeber function*/
    //initialize for the first step
    void ekf_init(uint64_t time,
        double vn,double ve,double vt,
        double lat,double lon,double alt,
        float p,float q,float r,
        float ax,float ay,float az,
        float hx,float hy,float hz);


    //  lla rate
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V, Eigen::Matrix<double,3,1> lla);
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V, double lat, double alt);

    // lla to ecef
    Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla);
    // ecef to ned
    Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef, Eigen::Matrix<double,3,1> pos_ref);
    //quaternion to dcm
    Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q);
    // quaternion multiplication
    Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q);
    // maps angle to +/- 180
    float constrainAngle180(float dta);
    // maps angle to 0-360
    float constrainAngle360(float dta);
    // Returns Radius - East West and Radius - North South
    constexpr std::pair<double, double> earthradius(double lat);
    // Yaw, Pitch, Roll to Quarternion
    Eigen::Matrix<float,4,1> toQuaternion(float yaw, float pitch, float roll);
    // Quarternion to Yaw, Pitch, Roll
    std::tuple<float, float, float> toEulerAngles(Eigen::Matrix<float,4,1> quat);
    // Update Jacobian matrix
    void updateJacobianMatrix();
    // Update Process Noise and Covariance Time
    void updateProcessNoiseCovarianceTime(float _dt);
    // Update Gyro and Accelerometer Bias
    void updateBias(float ax,float ay,float az,float p,float q, float r);
    // Update 15 states after KF state update
    void update15statesAfterKF();
    // Update differece between predicted and calculated GPS and IMU values
    void updateCalculatedVsPredicted();
    void ekf_update(uint64_t time);
    void updateINS();
};