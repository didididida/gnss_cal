/*
helper function for particle filter
*/

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_
#include <math.h>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <ros/ros.h>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <sensor_msgs/Imu.h>

#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif


// class for a point
class Point{
public:
   Point(double x,double y,double z):_x(x),_y(y),_z(z){
    }
   Point(){};
   double _x;
   double _y;
   double _z;
};

//class for a plane
class Plane{
public:
    Plane(double A,double B,double C,double D,double zmax,double zmin):_A(A),_B(B),_C(C),_D(D)
    ,z_max(zmax),z_min(zmin){
        initialize();
    }

    /*initialize a plane
      initialize the normal vector of plane
      initialize a point on the plane
    */
    void initialize(){
    squredSum = pow(_A,2)+pow(_B,2)+pow(_C,2);
    normalizer = sqrt(squredSum);
    normal[0] = _A/normalizer;
    normal[1] = _B/normalizer;
    normal[2] = _C/normalizer;
    p._x = _A*_D/squredSum;
    p._y = _B*_D/squredSum;
    p._z = _C*_D/squredSum;
    }

    double _A;
    double _B;
    double _C;
    double _D;
    double z_max;
    double z_min;
    double squredSum; 
    double normalizer;
    double normal[3];  //normal vector of this plane
    Point p;  //Point on the plane
};


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


/*get mirrored point of one known point to an known plane*/
Point mirror(const Point&point, const Plane&plane){
    Point symm_point;
    //calculate dot(P-p0,normal)
    double dott = plane.normal[0]*(point._x-plane.p._x)+plane.normal[1]*(point._y-plane.p._y)
    +plane.normal[2]*(point._z-plane.p._z);
    //Point R is the orthogonal projection of point on the plane
    double rx = point._x-plane.normal[0]*dott;
    double ry = point._y-plane.normal[1]*dott;
    double rz = point._z-plane.normal[2]*dott;
    symm_point._x = 2*rx-point._x;
    symm_point._y = 2*ry-point._y;
    symm_point._z = 2*rz-point._z;
    return symm_point;
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
/*
Below are functions about transformation between different coordinate system
const double d2r = M_PI/180.0;
const double r2d = 180.0/M_PI;
const double a = EARTH_SEMI_MAJOR; //Info is WSG-84
const double f_inverse = 298.257223563;
const double b = a - a/f_inverse;
const double e = sqrt(pow(a,2)-pow(b,2))/a;
const double epsilon = 0.000000000000001;

//From lla to ecef in WSG-84
void Blh2Xyz(double &x, double &y,double &z){
        double L = x * d2r;
        double B = y * d2r;
        double H = z;

      double N = a / sqrt(1- e*e*sin(B)*sin(B));
      x =(N+H) *cos(B)*cos(L);
      y = (N+H) * cos(B) * sin(L);
      z = z = (N * (1 - e * e) + H) * sin(B);
}

//From ecef to lla
void Xyz2Blh(double &x,double &y,double &z){
      double tmpX = x;
      double tmpY = y;
      double tmpZ = z;

      double curB = 0;
      double N = 0;
      double calB = atan2(tmpZ,sqrt(tmpX*tmpX + tmpY*tmpY));
      int counter = 0;
      while(abs(curB - calB)*r2d > epsilon && counter < 25 ){
          curB = calB;
          N = a / sqrt(1 - e * e * sin(curB) * sin(curB));
          calB = atan2(tmpZ + N * e * e * sin(curB), sqrt(tmpX * tmpX + tmpY * tmpY));
          counter++;
      }
      x = atan2 (tmpY,tmpX) * r2d;
      y = curB * r2d ;
      z = tmpZ / sin(curB) - N * (1 - e * e);	
}

//From ecef to enu 
void Ecef2Enu(Eigen::Vector3d &topocentricOrigin, Eigen::Vector4d& resultMat){
    double rzAngle  = (topocentricOrigin.x() * d2r + M_PI/2);
    Eigen::AngleAxisd rzAngelAxis(rzAngle,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d rZ = rzAngelAxis.matrix();

    double rxAngle = -(M_PI / 2 - topocentricOrigin.y() * d2r);
    Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rX = rxAngleAxis.matrix();

    Eigen::Matrix4d rotation;
    rotation.setIdentity();
    rotation.block<3, 3>(0, 0) = (rX * rZ);
      double tx = topocentricOrigin.x();
	  double ty = topocentricOrigin.y();
	  double tz = topocentricOrigin.z();
    Blh2Xyz(tx, ty, tz);
	  Eigen::Matrix4d translation;
  	translation.setIdentity();
	  translation(0, 3) = -tx;
	  translation(1, 3) = -ty;
	  translation(2, 3) = -tz;
	  resultMat = rotation * translation;
}

//From enu to ecef
void Enu2Ecef(Eigen::Vector3d &topocentricOrigin, Eigen::Matrix4d &resultMat){
     double rzAngle = (topocentricOrigin.x()*d2r + M_PI/2);
     Eigen::AngleAxisd rzAngelAxis(rzAngle,Eigen::Vector3d(0,0,1));
     Eigen::Matrix3d rZ = rzAngelAxis.matrix();

     double rxAngle = (M_PI/2 + topocentricOrigin.y()*d2r);
     Eigen::AngleAxisd rxAngleAxis(rxAngle,Eigen::Vector3d(1,0,0));
     Eigen::Matrix3d rX = rzAngelAxis.matrix();

     Eigen::Matrix4d rotation;
     rotation.setIdentity();
     rotation.block<3,3>(0,0) = rZ*rX;

     double tx = topocentricOrigin.x();
     double ty = topocentricOrigin.y();
     double tz = topocentricOrigin.z();
     Blh2Xyz(tx, ty ,tz);
     Eigen::Matrix4d translation;
     translation.setIdentity();
     translation (0,3) = tx;
     translation (1,3) = ty;
     translation (2,3) = tz;

     resultMat = translation * rotation;

}


//From one to local coordinate system, rostate by axis z
void Enu2local(double x,double y,const double theta,double &nx,double &ny){
    double nx = x;
    double ny = y;
    double rz = theta * d2r;
    nx = cos (rz) *nx - sin(rz)*ny;
    ny = sin(rz)*nx + cos(rz) * ny;
}
*/

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
#endif 