/*
helper function for particle filter
*/

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_
#include <math.h>
#include <sstream>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>

#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif


// class for a point
class Point{
public:
   Point(double x,double y,double z):_x(x),_y(y),_z(z){
    }
   Point();
   double _x;
   double _y;
   double _z;
};

//class for a plane
class Plane{
public:
    Plane(double A,double B,double C,double D):_A(A),_B(B),_C(C),_D(D){
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


#endif 