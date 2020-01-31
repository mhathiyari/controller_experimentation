#include "kf.hpp"

using namespace std;
namespace plt = matplotlibcpp;

class Dog {
    public:
    Point pos;
    float procces_var;
    float sensor_var;
    float velocity;
    std::default_random_engine generator(time(0));
    std::normal_distribution<double> distribution(0.0,1.0);
    Point getactual(){
        return pos;
    }
    Point getSensor(){
        Point result;
        result.x = pos.x +sensor_var*distribution(generator);
        result.y = pos.y;
    }
    void move(flaot dt){
        float dx = velocity + distribution(generator)*procces_var;
        pos.x += dx*dt; 
    }
    Point moveAndSense (flaot dt){
        move(dt);
        return getSensor();
    }
    Dog(){
        pos.x = 0;
        pos.y = 0;
        procces_var = 1.0;
        sensor_var = 2.0;
        velocity = 1;
    }
};


int main (){
    Dog dog;
    vector<double> x , time;
    for (int i =0; i<10; i++){
        x.push_back(dog.pos.x);
        time.push_back(i);
        dog.move(1);
    }
    plt::clf();
			// Plot line from given x and y data. Color is selected automatically.            
    plt::named_plot("Car",x,time,"*k");
    plt::show();
}