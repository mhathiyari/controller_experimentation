#include<eigen3/Eigen/Core>
#include<bits/stdc++.h>

{
using namespace Eigen;
using namespace std;

class quintic_polynomial{

double a0,a1,a2,a3,a4,a5;
void quintic_polynomial(double x_start,double v_start,double a_start,double x_end,double v_end,double a_end,double T){
           // calc coefficient of quintic polynomia
        // a0+a1*t+
        //then at t=0
        a0 = x_start;
        a1 = v_start;
        a2 = a_start / 2.0;
        
        MatrixXd A(3,3);
        A.row(0) << pow(T,3), pow(T, 4), pow(T, 5);
        A.row(1) << 3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4);
        A.row(2) << 6*T, 12*pow(T,2), 20*pow(T,3);

        MatrixXd b(3,1);
        b.col(0) << x_end - a0 - a1 * T - a2 * pow(T, 2),
                    v_end - a1 - 2 * a2 * T,
                    a_end - 2 * a2;

        x = A.colPivHouseholderQr().solve(b);

        a3 = x[0];
        a4 = x[1];
        a5 = x[2];
}
public:

double calc_point(t){
    return (a0 +a1 * t +a2 * pow(t,2) + 
            a3 * pow(t,3) +a4 * pow(t,4) +a5 * pow(t,5));
}
double calc_first_derivative(self, t){
    return (a1 + 2 *a2 * t + 3*a3 *pow(t,2)
             + 4*a4 *pow(t,3)+ 5*a5*pow(t,4));
}

double calc_second_derivative(self, t){
    return (2 *a2 + 6 *a3 * t + 12 *a4 * pow(t,2) + 20 *a5 * pow(t,3));
}

double calc_third_derivative(self, t){
    return (6 *a3 + 24 *a4 * t + 60 *a5 * pow(t,2));
}
};
}
 
