#include"mpc_steering.hpp"

using CppAD::AD;


int N = 7;
int target_speed = 15; //m/s
const double Lf = 2.67;
size_t x_start = 0;
size_t y_start = x_start+N;
size_t w_start = y_start+N;
size_t v_start = w_start+N;
size_t cte_start = v_start+N;
size_t we_start = cte_start+N;
size_t delta_start = we_start+N;
size_t a_start = delta_start+N-1;
double dt = 0.1;

class FG_eval{
    public:
    Eigen::VectorXd coeff;
    FG_eval(Eigen::VectorXd coeff){this->coeff = coeff;};

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& var){

        // fg[0] = 0;
        for(int t = 0;t<N;t++){

            //cost for cte
            fg[0] += 2000*CppAD::pow(var[cte_start+t],2);
            // cost for orientation errror
            fg[0] += 1000*CppAD::pow(var[we_start+t],2);
            //cost of target speed
            fg[0] += 2*CppAD::pow((var[v_start+t]-target_speed),2);
            
            // TODO add actuator cost

            //TODO  add shock cost

        }
        //contriant for the instial state
        fg[1+x_start] = var[x_start];
        fg[1+y_start] = var[y_start];
        fg[1+w_start] = var[w_start];
        fg[1+v_start] = var[v_start];
        fg[1+cte_start] = var[cte_start];
        fg[1+we_start] = var[we_start];
        //now we make sure that x-->x+1 can be reached through the integration of x 
        // this part has the dynamics and integration combined

        for(int t=1;t<N;t++){
            //current state
            AD<double> x0 = var[x_start+t-1];
            AD<double> y0 = var[y_start+t-1];
            AD<double> w0 = var[w_start+t-1];
            AD<double> v0 = var[v_start+t-1];
            AD<double> cte0 = var[cte_start+t-1];
            AD<double> we0 = var[we_start+t-1];
            AD<double> delta0 = var[delta_start+t-1];
            AD<double> a0 = var[a_start+t-1];
            //next state
            AD<double> x1 = var[x_start+t];
            AD<double> y1 = var[y_start+t];
            AD<double> w1 = var[w_start+t];
            AD<double> v1 = var[v_start+t];
            AD<double> cte1 = var[cte_start+t];
            AD<double> we1 = var[we_start+t];
            // AD<double> delta1 = var[delta_start+t];
            // AD<double> a1 = var[a_start+t];

            //calc ref point
            AD<double> f0 = coeff[0] + coeff[1]*x0 + coeff[2]*CppAD::pow(x0,2) + coeff[3]*CppAD::pow(x0,3);
            //cacl ref point angle
            AD<double> w_ref = CppAD::atan(coeff[1] + 2*coeff[2]*x0 + 3*coeff[3]*CppAD::pow(x0,2));
            // now we say that the next state - integrated current state should be 0
            fg[1+x_start+t] = x1 - (x0 +v0*CppAD::cos(w0)*dt);
            fg[1+y_start+t] = y1 - (y0 +v0*CppAD::sin(w0)*dt);
            fg[1+w_start+t] = w1 - (w0 +v0*delta0/Lf*dt);
            fg[1+v_start+t] = v1 - (v0 + a0*dt);
            fg[1+cte_start+t] = cte1 - ((f0-y0)+(v0*CppAD::sin(we0)*dt));
            fg[1+we_start+t] = w1 - ((w0-w_ref)+v0*delta0/Lf*dt);
        }
        // for(int i =0; i<fg.size();i++){
        //     std::cout<<fg[i]<<std::endl;
        // }
        //std::cout<<"inside FG_val";

    }

};


vector<double> MPC::mpc_solve(Eigen::VectorXd state,Eigen::VectorXd coeff){
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_var = N*6+(N-1)*2; // number of states for all time + number of control i/p for time-1
    size_t n_contriants = N*6; //to ensure that integration contraints are meet

    Dvector var(n_var);
    for(int i = 0;i<n_var;i++){
        var[i] = 0;
    }
    
    //copying states into CPPAD vector
    var[x_start] = state[0];
    var[y_start] = state[1];
    var[w_start] = state[2];
    var[v_start] = state[3];
    var[cte_start] = state[4];
    var[we_start] = state[5];
    //upper and lower limits for var(states)
    Dvector var_lowerbounds(n_var);
    Dvector var_upperbounds(n_var);

    for (int i = 0;i<delta_start;i++){
        var_lowerbounds[i] = -99999999;
        var_upperbounds[i] = 99999999;
    }

    for(int i = delta_start;i<a_start;i++){
        var_lowerbounds[i] = -0.4363;
        var_upperbounds[i] = 0.4363; // 25 deg to rad
    }

    for(int i=a_start;i<n_var;i++){
        var_lowerbounds[i] = -1;
        var_upperbounds[i] = 1;
    }

          

    Dvector constraints_lb(n_contriants); 
    Dvector constraints_up(n_contriants);
    for(int i=0;i<n_contriants;i++){
        constraints_up[i]=0;
        constraints_lb[i]=0;
    }

    constraints_lb[x_start] = state[0];
    constraints_lb[y_start] = state[1];
    constraints_lb[w_start] = state[2];
    constraints_lb[v_start] = state[3];
    constraints_lb[cte_start] = state[4];
    constraints_lb[we_start] = state[5];

    constraints_up[x_start] = state[0];
    constraints_up[y_start] = state[1];
    constraints_up[w_start] = state[2];
    constraints_up[v_start] = state[3];
    constraints_up[cte_start] = state[4];
    constraints_up[we_start] = state[5];

    FG_eval fg_eval(coeff);
// options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

    // std::string options;

    // options += "Sparse true             forward\n";
    // options += "Sparse true             reverse\n";
    // options += "Numeric max_cpu_time    0.5\n";

    CppAD::ipopt::solve_result<Dvector> result;
    // std::cout<<"calling ipopt";
    //********* calling the ipopt solver **********//
    CppAD::ipopt::solve(options,var,var_lowerbounds,var_upperbounds,constraints_lb,constraints_up,fg_eval,result);

    // // Parsing the result
    // std::cout << "Solution Status :" << result.status <<"/n";
    // std::cout << "cost :" << result.obj_value << "/n";
    // return control at the 1st timestamp

  bool ok = true;
  ok &= result.status == CppAD::ipopt::solve_result<Dvector>::success;
  std::cout<<"status" << ok << std::endl;
    std::vector<double> control;
    control.push_back(result.x[delta_start]);
    control.push_back(result.x[a_start]);
    std::cout<<"delta " << control[0] << std::endl;
    std::cout<<"a" << control[1] << std::endl;
    return control;

}


int calc_lookahead_pt(std::vector<double> cx,std::vector<double> cy,state veh){
    double min_dis = std::numeric_limits<double>::max();
    int indx = 0, i = 0;
    for(;i<cx.size();i++){
        double dx = veh.x - cx[i];
        double dy = veh.y - cy[i];
        double tempdist = sqrt(pow(dx,2)+pow(dy,2));
        if(min_dis>tempdist){
            min_dis = tempdist;
            indx = i;
        }
    }
    // double angle = pi2pi(cd[indx] - atan2(cy[indx] - veh.y, cx[indx] - veh.x));
    // // if(angle < 0) indx = -indx;
    // isRight = (angle < 0)? true : false; 
    return indx;
}

void get_14points(int indx,Eigen::VectorXd &x_14pts,Eigen::VectorXd &y_14pts,std::vector<double>cx,std::vector<double>cy){
    for(int i=0;i<7;i++){
        x_14pts[i] = cx[i+indx];
        y_14pts[i] = cy[i+indx];
    }
}

void transform_to_local(state veh_,Eigen::VectorXd &x_14pts,Eigen::VectorXd &y_14pts){
    Eigen::VectorXd pnt(2);
    Eigen::VectorXd local_pnt(2);

    Eigen::MatrixXd translation(2,2);
    translation <<  cos(-veh_.w), -sin(-veh_.w),
                    sin(-veh_.w),  cos(-veh_.w);
    for(int i =0; i<x_14pts.size();i++){
        pnt << x_14pts[i] - veh_.x, y_14pts[i] - veh_.y;
        local_pnt = translation * pnt;
        x_14pts[i] = local_pnt[0];
        y_14pts[i] = local_pnt[1];
    }
}



int main (){
    // generate trajectory
    std::vector<double> cx((50)/0.1,0.0);
    for(double i = 1; i < cx.size();i++){
        cx[i] = cx[i-1]+0.1;
    }
    std::vector<double> cy (cx.size(),0);
    for(double i = 0; i < cy.size();i++){
        cy[i] = sin(cx[i]/5.0)*cx[i]/2.0;//sin(ix / 5.0) * ix / 2.0 for ix in cx]
    }

    //initalize state
    // double _x, _y; 
    // std::cout << "enter x: " << endl;
    // std::cin >> _x;
    // std::cout << "enter y: " << endl;
    // std::cin >> _y; 
    state veh_ (0,0,0,0);
    // set speed
    std::vector<state> veh_state;
    veh_state.push_back(veh_);
    std::vector<double> x,y,w,v,t;

    int max_time = 100;
    double time = 0, delta = 0, a =1;
    //std::cout << "before while ";

    MPC controller;
    Eigen::VectorXd x_14pts(7);
    Eigen::VectorXd y_14pts(7);
    int last_target_idx = 0;
    while (time < max_time ){
        int clst_indx = calc_lookahead_pt(cx, cy,veh_); // find the closest point
        get_14points(clst_indx,x_14pts,y_14pts,cx,cy); //find the closest 14 points
        transform_to_local(veh_,x_14pts,y_14pts);
        //std::cout<<"14 points";

        // for(int i=0;i<x_14pts.size();i++){
        //     std::cout<<x_14pts[i]<<std::endl;
        //     std::cout<<y_14pts[i]<<std::endl;

        // }
        Eigen::VectorXd coeff = polyfit(x_14pts,y_14pts,3);
        // std::cout<<"start coeff";
        // for(int i=0;i<coeff.size();i++){
        //     std::cout<<coeff[i]<<std::endl;
        // }
        // std::cout<<"start coeff";

        double cte = coeff[0];
        double we = -atan(coeff[1]);
        std::cout<<we;
        Eigen::VectorXd vehstate(6);
        vehstate << 0, 0, 0, veh_.v, cte, we;

        auto res = controller.mpc_solve(vehstate,coeff);
        std::cout<<res[0]<<" " << res[1] << std::endl;
        veh_.update(res[1],res[0],dt);
        time = time + dt;
        veh_state.push_back(veh_);
        last_target_idx++;
        if(last_target_idx == cx.size()-14)
            break;
        #if PLOT
            x.push_back(veh_state.back().x);
            y.push_back(veh_state.back().y);
            // Clear previous plot
			plt::clf();
			// Plot line from given x and y data. Color is selected automatically.            
			plt::named_plot("Car",x,y,"*k");
            plt::named_plot("Track",cx,cy,"-r");   
            plt::legend();
            // plt::xlim(-1,60);
            // plt::plot(cx,cdd,"-b");
            plt::pause(0.01);

        #endif
    }
    plt::show(); 
    return 0;
}

//g++ mpc_steering.cpp -I/usr/include/python2.7 -lpython2.7 -lipopt -std=c++11
