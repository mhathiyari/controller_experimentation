#include "kf.hpp"


// class Dog {
//     public:
//     Point pos;
//     float procces_var;
//     float sensor_var;
//     float velocity;
//     float position_var = 200;
//     Point getactual(){
//         return pos;
//     }
//     Point getSensor(){
//         Point result;
//         result.x = pos.x +sensor_var*distribution(generator);
//         result.y = pos.y+sensor_var*distribution(generator);
//         return result;
//     }
//     void move(float dt){
//         float dx = velocity+procces_var*distribution(generator);
//         pos.x += dx*dt; 
//         float dy = velocity+procces_var*distribution(generator);
//         pos.y += dy*dt; 
//     }
//     Point moveAndSense (float dt){
//         if(dt !=0 )
//             move(dt);
//         return getSensor();
//     }
//     Dog(){
//         pos.x = 0;
//         pos.y = 0;
//         procces_var = 0.50;
//         sensor_var = 5.0;
//         velocity = 1;
//     }
// };
// struct gauss{
//     float mean;
//     float var;
//     gauss(float mean_ = 0,float var_ =0){
//         mean = mean_;
//         var = var_;
//     }
//     void operator=(gauss A){
//         mean = A.mean;
//         var = A.var;
//     }
// };
// class KF{
//     public:
//     KF(){};
//     gauss predict(gauss posterior,gauss movement){
//         gauss prior;
//         prior.mean = posterior.mean+movement.mean;
//         prior.var = posterior.var+movement.var;
//         return prior;
//     }
//     gauss update(gauss prior,gauss measurement){
//         gauss result;
//         float residual  = measurement.mean-prior.mean;
//         double K = prior.var/(measurement.var+prior.var);
//         result.mean = prior.mean+K*residual;
//         result.var = (1-K)*prior.var;
//         return result;
//     }
// };
/* to test the dynamics
int main (){
    Dog dog;
    vector<int> x;
    vector<double> measurex;
    vector<double> measurey;

        vector<double> actualx;
        vector<double> actualy;
        vector<double> time;
    for(double t=0;t<100;t+=0.5){
        Point sensor = dog.moveAndSense(0.5);
        measurex.push_back(sensor.x);
        measurey.push_back(sensor.y);
        actualx.push_back(dog.getactual().x);
        actualy.push_back(dog.getactual().y);

        time.push_back(t);
        }
    plt::clf();
			// Plot line from given x and y data. Color is selected automatically.            
    plt::named_plot("measure",measurex,measurey,"*k");
    plt::named_plot("actual",actualx,actualy,"+r");
    // plt::named_plot("op",opkf,time);
    plt::show();
    return 0;
}

*/