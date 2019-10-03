#include "purepursuit.hpp"

purepursuit::purepursuit(vector<double> cx_,vector<double> cy_,state veh_){
cx = cx_ ;
cy = cy_ ;
veh = veh_ ;
}

vector<double> purepursuit::pure_pursuit_control()
{
    int indx = calc_lookahead_pt();
    int tx=0,ty=0;
    if (pindx>=indx)
        indx = pindx;
    if (indx < cx.size()){
        tx = cx[indx];
        ty = cy[indx];
        cout<< "ok"<<endl;
    }else{
        tx = cx.back();
        ty = cy.back();
        indx = cx.size()-1;
    }
    
    double alpha = atan2(ty-veh.y_rear,tx-veh.x_rear)-veh.w;
    alpha = mod2pi(alpha);
    double look_ahead_dist = k*veh.v + Lfc;
    double delta = atan2f(2*L*sin(alpha)/look_ahead_dist,1.0);
    delta = mod2pi(delta);
    vector<double> result;
    double a = pid_vel();
    result.push_back(a);
    result.push_back(delta);
    pindx = indx;
    return result;

}

double purepursuit::pid_vel(){
    return kv*(target_speed-veh.v);
}

int purepursuit::calc_lookahead_pt(){
    int indx = 0;
    if (oldNeareastIndx == -1){
        int near = INT_MAX;
        for(int i = 0 ;i<cx.size();i++){
            int temp = veh.distance(cx[i],cy[i]);
            if(temp<near){
                near = temp;
                indx = i;
            }
        }
        oldNeareastIndx = indx;
    }else
    {
        indx = oldNeareastIndx;
        double distance_this_index = veh.distance_rear(cx[indx],cy[indx]);
        while(true){
            (indx+1<cx.size()) ? indx++ : indx;
            double distance_next_index = veh.distance_rear(cx[indx],cy[indx]);
            if(distance_this_index<distance_next_index)
                break;
            distance_this_index = distance_next_index;
        }
        oldNeareastIndx = indx;
    }   

    double Ld = 0;
    double Lf = k*veh.v+Lfc;

    while(Lf>Ld && indx+1<cx.size())
    {
        Ld = veh.distance_rear(cx[indx],cy[indx]);
        indx += 1;
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

    //initalize state
    state veh_ (0,3,0,0);
    // set speed
    purepursuit control(cx,cy,veh_);

    vector<state> veh_state;
    veh_state.push_back(veh_);
    vector<double> x,y,w,v,t;

    int max_time = 100;
    double time = 0,delta = 0,a=1,dt =0.1;
    while (time < max_time){
        vector<double> result = control.pure_pursuit_control();
        control.veh.update(result[0],result[1],dt);




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
}//g++ purepursuit.cpp -I/usr/include/python2.7 -lpython2.7 -I/home/mustafahathiyari/libraries/matplotlib-cpp -std=c++11
