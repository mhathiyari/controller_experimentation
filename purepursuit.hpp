#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>
#define PLOT (1)

 
using namespace std;
namespace plt = matplotlibcpp;

inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}


struct state
{
    double x;
    double y;
    double w;
    double v;
    double x_rear;
    double y_rear;
    double L = 2.9;
    state(){x =0;y=0;w = 0;v=0;x_rear=0;y_rear=0;}
    state(double x_,double y_, double w_,double v_){
        x = x_; y = y_; w = w_; v = v_;
        x_rear = x-(L/2*cos(w));
        y_rear = y-(L/2*sin(w));
    }
    //operator=(state& veh_)
    double distance (double xd , double yd){
        double dx = x-xd;
        double dy = y-yd;
        return sqrt(pow(dx,2)+pow(dy,2));
    }
    double distance_rear (double xd , double yd){
        double dx = x_rear-xd;
        double dy = y_rear-yd;
        return sqrt(pow(dx,2)+pow(dy,2));
    }
    void update(double a,double delta,double dt,double Ld){
     x =  x +  v * cos(w) * dt;
     y =  y +  v * sin(w) * dt;
     w =   w +  v / Ld * tan(delta) * dt;
     w = mod2pi(w);
     v =  v + a * dt;
     x_rear =  x - ((Ld / 2) * cos(  w));
     y_rear =  y - ((Ld / 2) * sin(  w));
    }
};

class purepursuit {
    private:
    int oldNeareastIndx = -1;
    int pindx = -1;
    double Lfc = 2,k =0.1, kv = 1,dt =0.1;
    double target_speed = 10.0 / 3.6; // [m/s]

    vector<double> cy;
    vector<double> cx;

    int calc_lookahead_pt();
    double pid_vel();

public:
double Ld = 0;
    state veh;
    purepursuit(vector<double> cx,vector<double> cy,state veh_);
    vector<double> pure_pursuit_control();
};


