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




/*get mirrored point of one known point to an known plane
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

*/
 

#endif 