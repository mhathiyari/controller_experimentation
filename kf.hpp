#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>
#include<random>
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