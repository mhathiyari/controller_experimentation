#include "purepursuit.hpp"

purepursuit::purepursuit(vector<float> cx_,vector<float> cy_,state veh_){
cx = cx_ ;
cy = cy_ ;
veh = veh_ ;
}

purepursuit::pure_pursuit_control()
{
    int indx = calc_lookahead_pt(veh,cx,cy);
    int tx=0,ty=0;
    if (pindx>=indx)
        indx = pindx;
    if (indx < cx.size()){
        tx = cx[indx];
        ty = cy[indx];
    }else{
        tx = cx.back();
        ty = cy.back();
        indx = cx.size()-1;
    }
    
    float alpha = atan2(ty-veh.y_rear,tx-veh.x_rear)-veh.w;
    alpha = mod2pi(alpha);
    float look_ahead_dist = k*veh.v + Lfc;
    float delta = atan2f(2*L*sin(alpha)/look_ahead_dist,1.0);
    delta = mod2pi(delta);
    vector<pair<float,float>> result;
    result.first = delta;
    float a = pid_vel();
    result.second = a;
    return result;

}

float purepursuit::pid_vel(){
    return kv*(target_speed-veh.v);
}

int purepursuit::calc_lookahead_pt(state veh,int pindx){
    int indx = 0;
    if (oldNeareastIndx == -1){
        int near = INT_MAX;
        for(int i = 0 ;i<cx.size();i++){
            int temp = veh.distance_rear(cx[i],cy[i]);
            if(temp<near){
                near = temp;
                indx = i;
            }
        }
        oldNeareastIndx = indx;
    }else
    {
        indx = oldNeareastIndx;
        float distance_this_index = veh.distance_rear(cx[indx],cy[indx]);
        while(true){
            (indx+1<cx.size()) ? indx++ : indx;
            float distance_next_index = veh.distance_rear(cx[indx],cy[indx]);
            if(distance_this_index<distance_next_index)
                break;
            distance_this_index = distance_next_index;
        }
        oldNeareastIndx = indx;
    }   

    float L = 0;
    float Lf = k*veh.v+Lfc;

    while(Lf>L && indx+1<cx.size())
    {
        L = veh.distance_rear(cx[indx],cy[indx]);
        indx += 1;
    }
    return indx;

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
    state veh_ (0,3,0,0);
    // set speed
    purepursuit control(cx,cy,veh);

    vector<state> veh_state;
    veh_state.push_back(veh_);
    vector<float> x,y,w,v,t;

    int max_time = 100;
    float time = 0,delta = 0,a=1,dt =0.1;
    int pindx = 0, indx = calc_lookahead_pt(veh,cx,cy,pindx);

    while (time < max_time){
        vector<pair<float,float>> result = pure_pursuit(indx,pindx,veh,cx,cy);
        control.veh.update(result.second,result.first,dt);




        cout<<veh.x-cx[indx] << " " << veh.y-cy[indx] << " " << time <<endl;
        time = time +dt;
        veh_state.push_back(control.veh);
        #if PLOT
            x.push_back(veh_state.back().x);
            y.push_back(veh_state.back().y);
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
