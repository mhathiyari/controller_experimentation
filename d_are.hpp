#include<eigen3/Eigen>
#include<vector>

using namespace Eigen;
MatrixXd solve_DARE(MatrixXd& A,MatrixXd& B,MatrixXd& Q,MatrixXd& R){
   // solve a discrete time_Algebraic Riccati equation (DARE) https://en.wikipedia.org/wiki/Algebraic_Riccati_equation for detials about the method 
    MatrixXd P = Q;
    int maxiter = 150;
    float eps = 0.01;
    At = A.transpose();
    Bt = B.transpose();
    for (int i=0;i<maxiter;i++)
    {
        P_new = At*P*A - (At*P*B)*(R+Bt*P*B).inverse()*Bt*P*A+Q;
        if(abs(P_new-P).maxCoeff()< eps)
            break;
        P=P_new;
    }
    return P;
    }

 double   dlqr(MatrixXd& A,MatrixXd& B,MatrixXd& Q,MatrixXd& R,MatrixXd& P,MatrixXd& eigvalues)
    {
    /*Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    */

    // first, try to solve the ricatti equation
    P = solve_DARE(A, B, Q, R);
    Bt = B.transpose();
    // compute the LQR gain
    K = (Bt*P*B + R).inverse() * (Bt*P*A);
    EigenSolver<MatrixXf> es;
    eigvalues = es.compute(A - (B * K),false);

    return K
    }
 double   dlqr(MatrixXd& A,MatrixXd& B,MatrixXd& Q,MatrixXd& R){
     MatrixXd P;
     MatrixXd eigvalues;
     return dlqr(A,B,Q,R,P,eigvalues);
 }
