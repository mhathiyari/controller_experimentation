#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>
#define PLOT (1)

 
using namespace std;
namespace plt = matplotlibcpp;

inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

class purepursuit {

int oldNeareastIndx = -1;
state veh;
float Lfc = 2,k =0.1,L = 2.9, kv = 1;
float target_speed = 10.0 / 3.6; // [m/s]

vector<float> cy;
vector<float> cx;

int calc_lookahead_pt(state veh,int pindx);
float pid_vel(float target,float vel);

public:
purepursuit(vector<float> cx,vector<float> cy,state veh_);
vector<pair<float,float>> pure_pursuit_control(int curr_indx,int pindx,state veh);
}

struct state
{
    float x;
    float y;
    float w;
    float v;
    float x_rear;
    float y_rear;
    state(float x_,float y_, float w_,float v_){
        x = x_; y = y_; w = w_; v = v_;
        x_rear = x-(L/2*cos(w));
        y_rear = y-(L/2*sin(w));
    }
    //operator=(state& veh_)
    float distance (float xd , float yd){
        float dx = x-xd;
        float dy = y-yd;
        return sqrt(pow(dx,2)+pow(dy,2));
    }
    float distance_rear (float xd , float yd){
        float dx = x_rear-xd;
        float dy = y_rear-yd;
        return sqrt(pow(dx,2)+pow(dy,2));
    }
    void update(float a,float delta,float dt){
     x =  x +  v * cos(w) * dt;
     y =  y +  v * sin(w) * dt;
     w =   w +  v / L * tan(delta) * dt;
     w = mod2pi(w);
     v =  v + a * dt;
     x_rear =  x - ((L / 2) * cos(  w));
     y_rear =  y - ((L / 2) * sin(  w));
    }

