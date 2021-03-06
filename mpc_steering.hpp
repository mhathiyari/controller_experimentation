#include<eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Householder>
#include<iostream>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>
#include "matplotlibcpp.h"

#define PLOT (1)
using namespace std;
namespace plt = matplotlibcpp;

// namespace AD = CppAD;
class MPC{
    public:
    MPC(){};
    ~MPC(){};
    std::vector<double> mpc_solve(Eigen::VectorXd state,Eigen::VectorXd coeff);
};
inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

struct state
{
    double x;
    double y;
    double w;
    double v;
    double L = 2.67;

    double steer_max = 45.0 / 180.0 * M_PI; 

    state(){x = 0;y = 0;w = 0;v = 0;}
    state(double x_,double y_, double w_,double v_){
        x = x_; y = y_; w = w_; v = v_;
    }
    //operator=(state& veh_)
    double distance (double xd , double yd){
        double dx = x-xd;
        double dy = y-yd;
        return sqrt(pow(dx,2)+pow(dy,2));
    }
    void update(double a,double delta,double dt){

    // delta = (delta >  steer_max)?  steer_max : delta; 
    // delta = (delta < -steer_max)? -steer_max : delta;   

    x =  x +  v * cos(w) * dt;
    y =  y +  v * sin(w) * dt;
    w =   w +  v / L * tan(delta) * dt;
    w = mod2pi(w);
    v =  v + a * dt;
    }
};
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}