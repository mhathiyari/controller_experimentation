#include<iostream>
#include<eigen3/Eigen/Dense>
using namespace Eigen;
class KalmanFilter{
    public:
    KalmanFilter( MatrixXd A,
            MatrixXd B,
            MatrixXd P,
            MatrixXd H,
            MatrixXd Q,
            MatrixXd R,
            double dt);
    //KalmanFilter();

    void init(double t0,VectorXd& x0);

    void update(VectorXd& y);
    VectorXd state() { return x_hat; };
    double time() { return t; };
    
    private:
    MatrixXd A,B,P,H,K,Q,R,P0,I;
    VectorXd x,x_hat;
    int m,n;
    int t0,t;
    double dt;
};