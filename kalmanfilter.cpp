#include"kalmanfilter.hpp"

KalmanFilter::KalmanFilter(
                double dt,
                const eig::MatrixXd& A,
                const eig::MatrixXd& P, 
                const eig::MatrixXd& Q,
                const eig::MatrixXd& H,
                const eig::MatrixXd& R 
):A(A), H(H) , Q(Q), R(R), P0(P),
m(H.rows()),n(A.rows()),dt(dt),
I(n,n),x_hat(n),x_hat_new(n){
    I.setIdentity(n,n);
}

void KalmanFilter::init(double t0,eig::VectorXd x0){
    x_hat = x0;
    this->t0 = t0;
    t = t0;
    P = P0;
    cout<<"n "<<n<<"m "<<m<<endl;
}

void KalmanFilter::update(const eig::VectorXd& z){
    // Predict
    x_hat_new = A*x_hat;

    P = A*P*A.transpose() + Q;
    // Update
    K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
    x_hat_new = x_hat_new + K*(z-H*x_hat_new);
    // cout<<"I "<< I.rows()<<I.cols()<< "K "<< K.rows()<<K.cols() << "H " << H.rows()<<H.cols() << endl;
    P = (I-K*H)*P;
    x_hat = x_hat_new;
    t += dt;
}


