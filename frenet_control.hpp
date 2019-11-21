#include<iostream>
#include<vector>
#include "matplotlibcpp.h"

struct Point{
  double x;
  double y;
  Point(const double x_ = 0,const double y_ = 0){
    x = x_;
    y = y_;
  }

  void print() const{
    cout << "X, Y: " << x << ", " << y << endl;
  }
 
  Point operator-(const Point& A) const{
    return Point(x-A.x, y-A.y);
  }
 
  Point operator+(const Point& A) const{
    return Point(x+A.x, y+A.y);
  }
 
  Point operator*(const double a) const{
    return Point(x*a, y*a);
  }
  bool operator==(const Point& A) const{
   return (x == A.x && y == A.y);
  }
   
  void operator=(const Point& A){
   x = A.x;
   y = A.y;
  }
};
struct Path{
    double s;       //course postion
    double d;       //lateral distance 
    double d_d;     //lateral speed
    double d_dd;    // lateral acceleration
    double s_d;     // speed
    double cd = 0;
    double cv = 0;
    double cf = 0;
}
typedef struct states{
    double x; 
    double y; //someday change this to point 
    double theta;
    double vy;
    double theta_dot;
    bool operator==(const states& A) const{
        return (x == A.x && y == A.y);
    }
    bool operator!=(const states& A) const{
        return (x != A.x || y != A.y);
    }
    void operator=(const states& A){
        x = A.x;
        y = A.y;
        theta = A.theta;
        vy = A.vy;
        theta_dot = A.theta_dot;
    }
    void SetCoord(Point& A){ // camelcase all func names
        x = A.x;
        y = A.y;
        theta = M_PI;
        vy = 0;
        theta_dot = 0;
    }
    void NoiseState(const States& st, const double& nx, const double& ny){
        x = st.x + nx; 
        y = st.y + ny; 
    }
    void RandomState(const double& Random){
        theta =  2*M_PI*Random;
        vy = 0;
        theta_dot = 0;
    }
    double Cost(const states& q2){
        return (sqrt(pow((x-q2.x),2)+pow((y-q2.y),2)));
    }
}States;

MAX_ROAD_WIDTH