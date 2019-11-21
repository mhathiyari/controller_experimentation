//
#include "frenet_control.hpp"
#include "quintic.hpp"
using namespace std;
namespace plt = matplotlibcpp;
vector<Path> calc_frenet_paths(Path curr){
    vector<Path> path_list;
    //generate path for each offset goal ||The Prong||
    for(int di =-MAX_ROAD_WIDTH; di <= MAX_ROAD_WIDTH ; di += D_ROAD_W){
        //lateral planning
        quintic_polynomial lqt (curr.d,curr.d_d,curr.d_dd,di,0,0,T);
        for(int t = MINT; t<=MAXT ; t +=DT){

        }
    }
}
vector<Path> frenet_optimal_traj(vector<double> csp, Path curr, vector<Point> ob){
    vector<Path> fplist;
    fplist = calc_frenet_paths(curr);

}
int main (){
    //waypoints 
    vector<double> wx{0.0, 10.0, 20.5, 35.0, 70.5};
    vector<double> wy{ 0.0, -6.0, 5.0, 6.5, 0.0};
    // obstacle lists
    vector<Point> ob ;
    ob.push_back(20.0, 10.0);
    ob.push_back(30.0, 6.0);
    ob.push_back(30.0, 8.0);
    ob.push_back(35.0, 8.0);
    ob.push_back(50.0, 3.0);

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy);

    // initial state
    Path curr;
    curr.s_d= 10.0 / 3.6;  // current speed [m/s]
    curr.s = 0.0;  // current course position
    curr.d = 2.0;  // current lateral position [m]
    curr.d_d = 0.0;  // current lateral speed [m/s]
    curr.d_dd = 0.0;  // current latral acceleration [m/s]
    
    int area = 20;
    int end = 1;
    while(end){
        vector<Path> path_opt = frenet_optimal_traj(csp,curr,ob);


    }

}