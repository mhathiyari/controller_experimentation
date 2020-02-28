/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

// #include <iostream>
// #include <Eigen/Dense>
#include <vector>

#include "kalmanfilter.hpp"
#include "kf.hpp"
using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char* argv[]) {

  int n = 4; // Number of states
  int m = 2; // Number of measurements

  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1,dt,0,0, 0,1,0,0, 0,0,1,dt, 0,0,0,1;
  C << 1,0,0,0, 0,0,1,0;

  // Reasonable covariance matrices
  Q << .05,.0,.0,.0, .05,.05,.0,.0, .0,.0,.0,.05, .0,.0,.05,.05;
  R << 5,0, 0,5;
  P << 500,0,0,0, 0,49,0,0, 0,0,500,0, 0,0,0,49;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A,P, Q, C, R);

  

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  x0 << 0,5,0,5;
  kf.init(0,x0);

    Dog dog;
    vector<double> measurex;
    vector<double> measurey;
    vector<double> actualx;
    vector<double> actualy;
    Eigen::VectorXd y(m);
    Eigen::VectorXd kfstate(n);
    vector<double> kfx;
    vector<double> kfy;

    vector<double> time;
    for(double t=0;t<100;t+=0.5){
        Point sensor = dog.moveAndSense(0.5);
        measurex.push_back(sensor.x);
        measurey.push_back(sensor.y);
        y<< measurex.back(),measurey.back();
        kf.update(y);
        actualx.push_back(dog.getactual().x);
        actualy.push_back(dog.getactual().y);
        kfstate = kf.state();
        kfx.push_back(kfstate[0]);
        kfy.push_back(kfstate[2]);
        time.push_back(t);
        }
    plt::clf();
			// Plot line from given x and y data. Color is selected automatically.            
    plt::named_plot("measure",measurex,measurey,"*k");
    plt::named_plot("actual",actualx,actualy,"r");
    plt::named_plot("kf",kfx,kfy,"b");
    // plt::named_plot("op",opkf,time);
    plt::show();
    return 0;
  // Feed measurements into filter, output estimated states
  // double t = 0;
  // Eigen::VectorXd y(m);
  // //std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state() << std::endl;
  // for(int i = 0; i < measurements.size(); i++) {
  //   t += dt;
  //   y << measurements[i];
  //   kf.update(y);
  //   std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
  //       << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
  // }

  // return 0;
}
//g++ kalman-test.cpp kalmanfilter.cpp -std=c++11
