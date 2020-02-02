#include "kf.hpp"


class Dog {
    public:
    Point pos;
    float procces_var;
    float sensor_var;
    float velocity;
    float position_var = 200;
    Point getactual(){
        return pos;
    }
    Point getSensor(){
        Point result;
        result.x = pos.x +sensor_var*distribution(generator);
        result.y = pos.y;
        return result;
    }
    void move(float dt){
        float dx = velocity+procces_var*distribution(generator);
        pos.x += dx*dt; 
    }
    Point moveAndSense (float dt){
        if(dt !=0 )
            move(dt);
        return getSensor();
    }
    Dog(){
        pos.x = 0;
        pos.y = 0;
        procces_var = 2.0;
        sensor_var = 5.0;
        velocity = 1;
    }
};
struct gauss{
    float mean;
    float var;
    gauss(float mean_ = 0,float var_ =0){
        mean = mean_;
        var = var_;
    }
    void operator=(gauss A){
        mean = A.mean;
        var = A.var;
    }
};
class KF{
    public:
    KF(){};
    gauss predict(gauss posterior,gauss movement){
        gauss prior;
        prior.mean = posterior.mean+movement.mean;
        prior.var = posterior.var+movement.var;
        return prior;
    }
    gauss update(gauss prior,gauss measurement){
        gauss result;
        float residual  = measurement.mean-prior.mean;
        double K = prior.var/(measurement.var+prior.var);
        result.mean = prior.mean+K*residual;
        result.var = (1-K)*prior.var;
        return result;
    }
};

int main (){
    Dog dog;
    gauss prior(0,1),op;
    KF kf;
    vector<double> x , time,measure_x,opkf,varkf;
    int time_prev = 0;
    gauss x_state;
    x_state.mean = dog.pos.x;
    x_state.var = dog.position_var;
    for (int i =0; i<100; i++){
        x.push_back(dog.pos.x);
        time.push_back(i);
        int dt = i-time_prev;
        double pos_new = dog.moveAndSense(dt).x;
        if(dt !=0){
        gauss proces(dog.velocity,dog.procces_var);
        prior = kf.predict(x_state,proces);
        gauss measure (pos_new,dog.sensor_var);
        op = kf.update(prior,measure);
        }
        opkf.push_back(op.mean);
        varkf.push_back(op.var);
        measure_x.push_back(pos_new);
        x_state = op;
    }
    // for(auto i : varkf)
    //     cout<<i<<endl;

    plt::clf();
			// Plot line from given x and y data. Color is selected automatically.            
   plt::named_plot("Car",x,time,"*k");
    plt::named_plot("measure",measure_x,time,"+r");
    plt::named_plot("op",opkf,time);
    plt::show();
    return 0;
}