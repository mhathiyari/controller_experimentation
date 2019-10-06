#include "stanley.hpp"

Stanley::Stanley(vector<double> cx_,vector<double> cy_,state veh_){
cx = cx_ ;
cy = cy_ ;
veh = veh_ ;
}

double Stanley::stanley_control(){

    int current_target_idx = calc_target_index();

    vector<double> front_axle_vec {-cos(veh.w + M_PI / 2),- sin(veh.yaw + M_PI /2)};
    vector<double> error_front_axle {fx - cx[current_target_idx],fy - cy[current_target_idx]};
    error_front_axle.dot(front_axle_vec);

    if (last_target_idx >= current_target_idx)
        current_target_idx = last_target_idx;

    //theta_e corrects the heading error
    double theta_e = mod2pi(cd[current_target_idx] - veh.w);
    // theta_d corrects the cross track error
    double theta_d = atan2(k * error_front_axle, state.v);
    //Steering control
    double delta = theta_e + theta_d;
    last_target_idx = current_target_idx;
    return delta;
}
int Stanley::calc_lookahead_pt(){
    float fx = veh.x+cos(veh.w)*L;
    float fy = veh.y+sin(veh.w)*L;
    int min_dis = INT_MAX;
    int indx = 0;
    for(int i=0;i<cx.size();i++){
        float dx = fx - cx[i];
        float dy = fy - cy[i];
        float tempdist = sqrt(pow(dx,2)+pow(dy,2));
        if(min>tempdist){
            min = tempdist;
            indx = i;
        }
    }

    return indx;
}

int main(){
    // generate trajectory
    vector<double> cx((50)/0.1,0.0);
    for(double i = 1; i< cx.size();i++){
        cx[i] = cx[i-1]+0.1;
    }
    vector<double> cy (cx.size(),0);
    for(double i = 1; i< cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }
    vector<double> cd (cx.size(),0);
    for(double i = 1; i< cy.size();i++){
        cd[i] = atan2(cy,1);//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }

    //initalize state
    state veh_ (0,3,0,0);
    // set speed
    Stanley control(cx,cy,veh_);

    vector<state> veh_state;
    veh_state.push_back(veh_);
    vector<double> x,y,w,v,t;

    int max_time = 100;
    double time = 0,delta = 0,a=1,dt =0.1;
    while (time < max_time){
        double delta = control.stanley_control();
        double a = control.pid_vel();
        control.veh.update(a,delta,dt);
        //cout<<veh.x-cx[indx] << " " << veh.y-cy[indx] << " " << time <<endl;
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
}