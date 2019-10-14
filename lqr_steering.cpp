#include"lqr_steering.hpp"

LQRSteer::LQRSteer(vector<double> cx_,vector<double> cy_,vector<double> cd_,vector<double> cdd_,state veh_){
cx = cx_ ;
cy = cy_ ;
veh = veh_ ;
cd = cd_;
cdd = cdd_;
int indx = calc_lookahead_pt();
float dx = veh.x - cx[indx];
float dy = veh.y - cy[indx];
pe = sqrt(pow(dx,2)+pow(dy,2));
pth_e =  mod2pi(veh.w - cd[indx]);

}
int LQRSteer::calc_lookahead_pt(){
    int min_dis = INT_MAX;
    int indx = 0;
    for(int i=0;i<cx.size();i++){
        float dx = veh.x - cx[i];
        float dy = veh.y - cy[i];
        float tempdist = sqrt(pow(dx,2)+pow(dy,2));
        if(min_dis>tempdist){
            min_dis = tempdist;
            indx = i;
        }
    }

    return indx;
}

vector<double> LQRSteer::lqr_control(vector<double> sp,MatrixXd Q,MatrixXd R){
    int indx = calc_lookahead_pt();

    float tv = 10/3.6;//can be replaced by a speed profile

    double k = cdd[indx];
    double v = veh.v;
    th_e = mod2pi(veh.w - cd[indx]);

    //A = [1.0, dt, 0.0, 0.0, 0.0
    //     0.0, 0.0, v, 0.0, 0.0]
    //     0.0, 0.0, 1.0, dt, 0.0]
    //     0.0, 0.0, 0.0, 0.0, 0.0]
    //     0.0, 0.0, 0.0, 0.0, 1.0]
    MatrixXd A = MatrixXd::Zero(5, 5);
    A(0, 0) =1.0;
    A(0, 1) =dt;
    A(1, 2) =v;
    A(2, 2) =1.0;
    A(2, 3) =dt;
    A(4, 4) =1.0;

    //B = [0.0, 0.0
    //    0.0, 0.0
    //    0.0, 0.0
    //    v/L, 0.0
    //    0.0, dt]
    MatrixXd B = MatrixXd::Zero(5, 2);
    B(3, 0) =v / L;
    B(4, 1) =dt;

    MatrixXd K = dlqr(A, B, Q, R);

    //state vector
    //x = [e, dot_e, th_e, dot_th_e, delta_v]
    //e: lateral distance to the path
    //dot_e: derivative of e
    //th_e: angle difference to the path
    //dot_th_e: derivative of th_e
    //delta_v: difference between current speed and target speed
    float dx = veh.x - cx[indx];
    float dy = veh.y - cy[indx];
    e = sqrt(pow(dx,2)+pow(dy,2));
    MatrixXd x = MatrixXd::Zero(5, 1);
    x(0, 0) =e;
    x(1, 0) =(e - pe) / dt;
    x(2, 0) =th_e;
    x(3, 0) =(th_e - pth_e) / dt;
    x(4, 0) =v - tv;

    //input vector
    //u = [delta, accel]
    //delta: steering angle
    //accel: acceleration
    MatrixXd ustar = -K * x;
    cout<< ustar.size();
    //calc steering input
    double ff = atan2(L * k, 1);  //feedforward steering angle
    double fb = mod2pi(ustar(0, 0));  //feedback steering angle
    double delta = ff + fb;
    // delta = mod2pi(delta);
    //calc accel input
    double accel = ustar(1, 0);
    vector<double> res;
    res.push_back(delta);
    res.push_back(accel);
    pe = e;
    pth_e = th_e;
    return res;
}


int main (){
    // generate trajectory
    vector<double> cx((50)/0.1,0.0);
    for(double i = 1; i< cx.size();i++){
        cx[i] = cx[i-1]+0.1;
    }
    vector<double> cy (cx.size(),0);
    for(double i = 0; i< cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }
        vector<double> cd (cx.size(),0);
    for(double i = 0; i< cy.size();i++){
        double temp = cos(cx[i]/5.0)*cx[i]/10+sin(cx[i]/5.0)/2;

        cd[i] = atan2(temp,1);//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }
    vector<double> cdd (cx.size(),0);
    for(double i = 0; i< cy.size();i++){
        double temp = 0.2*cos(cx[i]/5.0)+sin(cx[i]/5.0)*0.02;
        cdd[i] = temp;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }

    //initalize state
    state veh_ (0,3,0,0);
    // set speed
    LQRSteer control(cx,cy,cd,cdd,veh_);

    vector<state> veh_state;
    veh_state.push_back(veh_);
    vector<double> x,y,w,v,t;
    Eigen::MatrixXd Q;
    Q.setIdentity(5,5);
    Eigen::MatrixXd R;
    R.setIdentity(2,2);
    vector<double> sp (5,0);
    int max_time = 100;
    double time = 0,delta = 0,a=1,dt =0.1;
    control.last_target_idx = control.calc_lookahead_pt();
    // plt::plot(x,y,"-k");
    // plt::pause(10);
    // plt::plot(cx,cy);
    // plt::pause(3);
    while (time < max_time){
        vector<double> res = control.lqr_control(sp,Q,R);
        // double a = control.pid_vel();
        control.veh.update(res[1],res[0],dt);
        time = time +dt;
        veh_state.push_back(control.veh);
        if(control.last_target_idx == cx.size()-1)
            break;
        #if PLOT
            x.push_back(veh_state.back().x);
            y.push_back(veh_state.back().y);
        // Clear previous plot
			plt::clf();
			// Plot line from given x and y data. Color is selected automatically.
			plt::named_plot("Car",x,y,"-k");
            plt::named_plot("Track",cx,cy,"-r");
            plt::legend();
            plt::xlim(-1,60);
            // plt::plot(cx,cdd,"-b");
            plt::pause(0.01);

        #endif
    }
    return 0;
}