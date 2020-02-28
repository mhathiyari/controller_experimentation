#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>
#include<random>
#include<time.h>
#include<vector>
#include<eigen3/Eigen/Dense>

using namespace std;
namespace plt = matplotlibcpp;
//namespace Eigen = eig;

default_random_engine generator;
normal_distribution<double> distribution(0.0,1.0);
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

class Dog {
    public:
    Point pos;
    float procces_var;
    float sensor_var;
    float velocity;
    float position_var = 200;
    Point getactual(){
        return pos;
    }
    Point getSensor(){
        Point result;
        result.x = pos.x +sensor_var*distribution(generator);
        result.y = pos.y+sensor_var*distribution(generator);
        return result;
    }
    void move(float dt){
        float dx = velocity+procces_var*distribution(generator);
        pos.x += dx*dt; 
        float dy = velocity+procces_var*distribution(generator);
        pos.y += dy*dt; 
    }
    Point moveAndSense (float dt){
        if(dt !=0 )
            move(dt);
        return getSensor();
    }
    Dog(){
        pos.x = 0;
        pos.y = 0;
        procces_var = 0.50;
        sensor_var = 5.0;
        velocity = 1;
    }
};