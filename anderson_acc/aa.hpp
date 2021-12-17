#pragma once
/**
    Anderson Acceleration for linear iterative problems.
    @author @copyright sentinel (Github: Enigmatisms)
    @ref (paper) Convergence Analysis for Anderson Acceleration
*/
#include <Eigen/Dense>
#include <iostream>
#include <deque>

template <typename T, int Dim>
class AA {
using MatrixDxt = Eigen::Matrix<T, Dim, -1>;
using MatrixDt = Eigen::Matrix<T, Dim, Dim>;
using MatrixXt = Eigen::Matrix<T, -1, -1>;
using VectorDt = Eigen::Matrix<T, Dim, 1>;
using VectorXt = Eigen::Matrix<T, -1, 1>;
public:
    AA(T restart_thresh, T alpha_lim = 10., int dim = 2): 
            restart_thresh(restart_thresh), alpha_lim(alpha_lim), dim(dim) {}
    ~AA() {}

    bool andersonAccelerate(const VectorDt& pose, const VectorDt& prev_pose, VectorDt& out_pose, T avg_err, int iter_num);
private:
    const T restart_thresh;
    const T alpha_lim;
    const int dim;
    std::deque<VectorDt> Gs;
    std::deque<VectorDt> Fs;
};
