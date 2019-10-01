#include "purepursuit.hpp"

purepursuit::purepursuit(vector<float> cx_,vector<float> cy_,state veh_){
cx = cx_ ;
cy = cy_ ;
veh = veh_ ;
}

purepursuit::pure_pursuit_control()
{
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
    
    float alpha = atan2f(ty-veh.y_rear,tx-veh.x_rear)-veh.w;
    alpha = mod2pi(alpha);
    float look_ahead_dist = k*veh.v + Lfc;
    float delta = atan2f(2*L*sin(alpha)/look_ahead_dist,1.0);
    delta = mod2pi(delta);
    return delta;

}

int main(){
    // generate trajectory
    vector<float> cx((50)/0.1,0.0);
    for(float i = 1; i< cx.size();i++){
        cx[i] = cx[i-1]+0.1;
    }
    vector<float> cy (cx.size(),0);
    for(float i = 1; i< cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }

    //initalize state
    state veh (0,3,0,0);
    // set speed
    purepursuit control(cx,cy,veh);

    vector<state> veh_state;
    veh_state.push_back(veh);
    vector<float> x,y,w,v,t;

    int max_time = 100;
    float time = 0,delta = 0,a=1,dt =0.1;
    int pindx = 0, curr_indx = calc_lookahead_pt(veh,cx,cy,pindx);

    while (time < max_time){
        delta = pure_pursuit(curr_indx,pindx,veh,cx,cy);
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
