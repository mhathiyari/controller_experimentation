#include<iostream>
#include<eigen3/Eigen/Dense>

#pragma once

using namespace std;
namespace eig = Eigen;

class KalmanFilter{
    eig::MatrixXd A,P,Q,H,R,I,K,P0;

    eig::VectorXd x_hat,x_hat_new;

    double t0,t,dt;

    int m,n;

    public:
    KalmanFilter(
                double dt,
                const eig::MatrixXd& A,
                const eig::MatrixXd& P, 
                const eig::MatrixXd& Q,
                const eig::MatrixXd& H,
                const eig::MatrixXd& R   
    );

    void init(double t0,eig::VectorXd x0);
    void update(const eig::VectorXd& z);
    eig::VectorXd state(){
        return x_hat;
    };

    double time(){return t;}
};