#include"matplotlibcpp"
#include<eigen3/Eigen/Core>
#include<iostream>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>

using namespace CppAD::AD;
class MPC{
    public:
    MPC(){};
    ~MPC(){};
    vector<double> mpc_solve(vector<double> state);
}
