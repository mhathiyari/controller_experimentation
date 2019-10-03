#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>

#define PLOT (1)
double Lfc = 2,k =0.1,L = 2.9, kv = 1;
using namespace std;
namespace plt = matplotlibcpp;

struct state
{
    double x;
    double y;
    double w;
    double v;
    double x_rear;
    double y_rear;
    state(double x_,double y_, double w_,double v_){
        x = x_; y = y_; w = w_; v = v_;
        x_rear = x-(L/2*cos(w));
        y_rear = y-(L/2*sin(w));
    }
    double distance (double xd , double yd){
        double dx = x-xd;
        double dy = y-yd;
        return sqrt(dx*dx+dy*dy);
    }
    double distance_rear (double xd , double yd){
        double dx = x_rear-xd;
        double dy = y_rear-yd;
        return sqrt(dx*dx+dy*dy);
    }
    void update(double a,double delta,double dt){
     x =  x +  v * cos(w) * dt;
     y =  y +  v * sin(w) * dt;
     w =   w +  v / L * tan(delta) * dt;
     //w = mod2pi(w);
     v =  v + a * dt;
     x_rear =  x - ((L / 2) * cos(  w));
     y_rear =  y - ((L / 2) * sin(  w));
    }
};
inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

double pure_pursuit(int curr_indx,int pindx,state veh,vector<double> cx,vector<double> cy){
    int tx=0,ty=0;
    if (pindx>=curr_indx)
        curr_indx = pindx;
    if (curr_indx < cx.size()){
        tx = cx[curr_indx];
        ty = cy[curr_indx];
    }else{
        tx = -1;
        ty = -1;
        //indx = cx.size()-1;
    }
    double alpha = atan2f(ty-veh.y_rear,tx-veh.x_rear)-veh.w;
    alpha = mod2pi(alpha);
    double look_ahead_dist = k*veh.v + Lfc;
    double delta = atan2f(2*L*sin(alpha)/look_ahead_dist,1.0);
    delta = mod2pi(delta);
    return delta;

}
int calc_lookahead_pt(state veh,vector<double> cx,vector<double> cy,int pindx){
    static int old_nearest_point = 0;
    int indx = 0;
    if(pindx ==0){
        double min_distance = FLT_MAX;
        for(int i = 0;i<cx.size();i++){
            double distance = veh.distance_rear(cx[i],cy[i]);
            if(min_distance > distance){
                min_distance = distance;
                indx = i;
                old_nearest_point = indx;
            }
        }
    }else
    {
        indx = old_nearest_point;
        double distance_now = veh.distance_rear(cx[indx],cy[indx]);
        while(1){
            indx = indx+1<cx.size() ? indx++ : indx;
            double distance_new = veh.distance_rear(cx[indx],cy[indx]);
            if(distance_now<distance_new) break;
            distance_now = distance_new;
        }
        old_nearest_point = indx;
    }
    double temp =0;
    double look_ahead_dist = k*veh.v + Lfc;
    while (look_ahead_dist > temp && indx+1 < cx.size()){
        temp = veh.distance_rear(cx[indx],cy[indx]);
        indx ++;
    }
    return indx;  
}

double pid_vel(double target,double vel){
    return kv*(target-vel);
}
int main(){
    cout<< "while";

    // generate trajectory
    vector<double> cx((50)/0.1,0.0);
    for(double i = 1; i< cx.size();i++){
        cx[i] = cx[i-1]+0.1;
    }
    vector<double> cy (cx.size(),0);
    for(double i = 1; i< cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }
    state veh (0,3,0,0);
    vector<state> veh_state;
    veh_state.push_back(veh);
    double target_speed = 10.0 / 3.6; // [m/s]

    int max_time = 100;
    double time = 0,delta = 0,a=1,dt =0.1;
    int pindx = 0, curr_indx = calc_lookahead_pt(veh,cx,cy,pindx);

    vector<double> x,y,w,v,t;
    while (time < max_time && curr_indx<cx.size()){
        pindx = curr_indx;
        cout<< "while";
        curr_indx = calc_lookahead_pt(veh,cx,cy,pindx);
        cout<< "index";        
        delta = pure_pursuit(curr_indx,pindx,veh,cx,cy);
        cout<< "purs";        
        a = pid_vel(target_speed, veh.v);
        veh.update(a,delta,dt);
        cout<<veh.x-cx[curr_indx] << " " << veh.y-cy[curr_indx] << " " << time <<endl;
        time = time +dt;
        veh_state.push_back(veh);
        #if PLOT
            x.push_back(veh.x);
            y.push_back(veh.y);
        // Clear previous plot
			plt::clf();
			// Plot line from given x and y data. Color is selected automatically.
			plt::plot(x,y,"-k");
            plt::plot(cx,cy,"-r");
            plt::pause(0.01);

        #endif
    }
    return 0;
}//g++ pure_pursuit.cpp -I/usr/include/python2.7 -lpython2.7 -I/home/mustafahathiyari/libraries/matplotlib-cpp -std=c++11
