#include"mpc_steering.hpp"

int N = 7;
int target_speed = 15; //m/s
size_t x_start = 0;
size_t y_start = x_start+N;
size_t w_start = y_start+N;
size_t v_start = w_start+N;
size_t cte_start = v_start+N;
size_t we_start = cte_start+N;
size_t delta_start = we_start+N-1;
size_t a_start = delta_start+N-1;

class FG_eval{
    Eigen::VectorXd coeff;
    FG_eval(Eigen::VectorXd coeff){this->coeff = coeff;}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, ADvector& var){

        for(int t = 0;t<N;t++){

            fg[0] = 0;
            //cost for cte
            fg[0] += 20*AD::pow(var[cte_start+t],2);
            // cost for orientation errror
            fg[0] += 100*AD::pow(var[we_start+t],2);
            //cost of target speed
            fg[0] += 2*AD::pow(var[v_start+t]-target_speed,2);
            
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
            AD<double> x0 = vars[x_start+t-1];
            AD<double> y0 = var[y_start+t-1];
            AD<double> w0 = var[w_start+t-1];
            AD<double> v0 = var[v_start+t-1];
            AD<double> cte0 = var[cte_start+t-1];
            AD<double> we0 = var[we_start+t-1];
            AD<double> delta0 = vars[delta_start+t-1];
            AD<double> a0 = vars[a_start+t-1];
            //next state
            AD<double> x1 = vars[x_start+t];
            AD<double> y1 = var[y_start+t];
            AD<double> w1 = var[w_start+t];
            AD<double> v1 = var[v_start+t];
            AD<double> cte1 = var[cte_start+t];
            AD<double> we1 = var[we_start+t];
            AD<double> delta1 = vars[delta_start+t];
            AD<double> a1 = vars[a_start+t];

            //calc ref point
            AD<double> f0 = 0;
            //cacl ref point angle
            AD<double> w_ref = 0;
            // now we say that the next state - integrated current state should be 0
            fg[1+x_start+t] = x1 - (x0 +v0*AD::cos(w)*dt);
            fg[1+y_start+t] = y1 - (y0 +v0*AD::sin(w)*dt);
            fg[1+w_start+t] = w1 - (w0 +v0*delta0/Lf*dt);
            fg[1+v_start+t] = v1 - (v0 + a0*dt);
            fg[1+cte_start+t] = cte1 - ((f0-y0)+(v0*AD::sin(we0)*dt));
            fg[1+we_start+t] = a1 - (w0-w_ref+v0*delta0/Lf*dt);
        }

    }

}


MPC::mpc_solve(Eigen::VectorXd state){
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t n_var = N*6+(N-1)*2; // number of states for all time + number of control i/p for time-1
    size_t n_contriants = N*6; //to ensure that integration contraints are meet

    Dvector var(n_var);
    for(int i = 0;i<nvar;i++){
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
    Dvector var_lowerbounds(n_vars);
    Dvector var_upperbounds(n_vars);

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

    var_lowerbounds[x_start] = state[0];
    var_lowerbounds[y_start] = state[1];
    var_lowerbounds[w_start] = state[2];
    var_lowerbounds[v_start] = state[3];
    var_lowerbounds[cte_start] = state[4];
    var_lowerbounds[we_start] = state[5];

    var_upperbounds[x_start] = state[0];
    var_upperbounds[y_start] = state[1];
    var_upperbounds[w_start] = state[2];
    var_upperbounds[v_start] = state[3];
    var_upperbounds[cte_start] = state[4];
    var_upperbounds[we_start] = state[5];

    Dvector constraints_lb(n_contriants), constriants_up(n_contriants);
    for(int i=0;i<n_contriants;i++){
        constriants_up[i]=0;
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

    std::string options;

    options += "Sparse true        forward\n";
    options += "Sparse true        reverse\n";
    options += "Numeric max_cpu_time    0.5\n"

}