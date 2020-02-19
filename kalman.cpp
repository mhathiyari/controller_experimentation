#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    MatrixXd A,
    MatrixXd B,
    MatrixXd P,
    MatrixXd H,
    MatrixXd Q,
    MatrixXd R,
    double dt)
    : A(A),B(B),P0(P),H(H),Q(Q),R(R),
     dt(dt), m(H.rows()),n(A.rows()),
     x_hat(n,1),x(n,1),I(n,n){
         I.setIdentity();
     }

void KalmanFilter::init(double t0,VectorXd& x0){
t = t0;
x = x0;
this->t0 = t0;
P = P0;
}
    
void KalmanFilter::update(VectorXd& y){
    x_hat = A*x ;
    P = A*P*A.transpose() + Q;
    VectorXd residual = y-H*x_hat;
    K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
    x_hat = x_hat+K*residual;
    P = (I-K*H)*P;
    x = x_hat;
    t +=dt;
}