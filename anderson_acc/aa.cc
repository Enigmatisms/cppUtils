#include "aa.hpp"

template <typename T, int Dim>
bool AA<T, Dim>::andersonAccelerate(const VectorDt& pose, const VectorDt& prev_pose, VectorDt& out_pose, T avg_err, int iter_num) {
    VectorDt fn = pose - prev_pose;
    if (avg_err > restart_thresh) {
        Gs.clear();
        Fs.clear();
        Gs.push_back(pose);
        Fs.push_back(-fn);
        return false;
    }
    if (Fs.size() > 0) 
        Fs.back() += fn;            // Calculate F(u_{k-m_k+j+1}) - F(u_{k-m_k+j}) directly
    Fs.push_back(-fn);
    Gs.push_back(pose);
    if (Fs.size() > dim + 1) {
        Fs.pop_front();
        Gs.pop_front();
    } else if (Fs.size() < 2) {
        return false;
    }
    // ========== the code above adds new fn and gn ============
    const int f_cols = Fs.size() - 1;
    VectorDt output = Gs.back();
    int f_col = 1;
    for (; f_col <= f_cols; f_col++) {
        MatrixDxt P(dim, f_col);
        VectorXt theta;
        for (int i = 0; i < f_col; i++)
            P.col(i) = Fs[f_cols - f_col + i];
        bool ill_conditioned = false;
        if (f_col < dim) {
            const MatrixXt M = P.transpose() * P;
            if (abs(M.determinant()) < 1e-9)
                ill_conditioned = true;
            else
                theta = M.inverse() * P.transpose() * fn;
        } else {
            if (abs(P.determinant()) < 1e-9)      // ill-conditioned
                ill_conditioned = true;
            else
                theta = P.colPivHouseholderQr().solve(fn);
        }
        if (ill_conditioned) {
            Gs.clear();
            Fs.clear();
            Gs.push_back(pose);
            Fs.push_back(-fn);
            return false;
        }
        const T a_k = 1 - theta(f_col - 1);
        if (a_k < 0)
            break;
        VectorXt alpha(f_col + 1);      // max size: dim + 1
        alpha(0) = theta(0);
        alpha(f_col) = a_k;
        for (int i = 1; i < f_col; i++)
            alpha(i) = theta(i) - theta(i - 1);
        if (alpha.maxCoeff() > alpha_lim || alpha.minCoeff() < -alpha_lim)
            break;
        output.setZero();
        for (int pos = f_col; pos >= 0; pos--)
            output += alpha(pos) * Gs[f_cols - f_col + pos];
    }
    return true;
}
