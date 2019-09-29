#include<bits/stdc++.h>
#include "matplotlibcpp.h"
#include <cmath>
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
    float distance (float xd , float yd){
        float dx = x-xd;
        float dy = y-yd;
        return sqrt(dx*dx+dy*dy);
    }
    float distance_rear (float xd , float yd){
        float dx = x_rear-xd;
        float dy = y_rear-yd;
        return sqrt(dx*dx+dy*dy);
    }
    void update(float a,float delta,float dt){
     x =  x +  v * cos(w) * dt;
     y =  y +  v * sin(  w) * dt;
     w =   w +  v / L * tan(delta) * dt;
     v =  v + a * dt;
     x_rear =  x - ((L / 2) * cos(  w));
     y_rear =  y - ((L / 2) * sin(  w));
    }
}

float pure_pursuit(state veh,vector<float> cx,vector<float>cy,int pindx){
    int curr_indx = calc_lookahead_pt(veh,cx,cy,pindx);

}
int calc_lookahead_pt(state veh,vector<float> cx,vector<float>cy,int pindx){
    static int old_nearest_point = 0;
    int indx = 0
    if(pindx ==0){
        float min_distance = FLT_MAX;
        for(int i = 0;i<num.size();i++){
            float distance = veh.distance(cx[i],cy[i]);
            if(min_distance > distance){
                min_distance = distance;
                indx = i;
                old_nearest_point = indx;
            }
        }
    }else
    {
        indx = old_nearest_point;
        float distance_now = veh.distance(cx[indx],cy[indx]);
        while(1){
            indx+1<cx.size() ? indx++ : indx;
            float distance_new = veh.distance(cx[indx],cy[indx]);
            if(distance_now<distance_new) break;
            distance_now = distance_new;
        }
        old_nearest_point = indx;
    }
    float k =1,temp =0;//gain on vel
    float look_ahead_dist = k*veh.v + 10;
    while (look_ahead_dist > temp){
        temp = veh.distance_rear(cx[indx],cy[indx]);
        indx ++;
    }
    return indx;
    
}
int main(){
    // generate trajectory
    vector<float> cx((0-50)/0.1,0);
    for(float i = 1; i< cx.size();i++){
        cx[i] = c[i-1]+0.1;
    }
    vector<float> cy (cx.size(),0);
    for(float i = 1; i< cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }
    state veh (0,3,0,0);
    vector<state> veh_state;
    veh_state.push_back(veh);
    int max_time = 100;
    float time = 0,delta = 0,a=1,dt =0.1;
    while time < max_time{
        delta = pure_pursuit(veh,cx,cy,indx);
        a = pid_vel(target_speed, veh.v);
        veh.update(a,delta,dt);
        time = time +dt;
        veh_state.push_back(veh);
    }
    return 0;
}