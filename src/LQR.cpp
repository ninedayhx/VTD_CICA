/**
 * @file LQR.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "LQR.h"

LQR::LQR(std::string _name)
{
    name = _name;
}

LQR::~LQR()
{
}

void LQR::get_param(Eigen::MatrixXf _Q, Eigen::MatrixXf _R, float ts)
{
    dim_x = _Q.rows();
    dim_u = _R.rows();
    P.resize(dim_x, dim_x);
    P_next.resize(dim_x, dim_x);
    K.resize(dim_x, dim_u);

    Q = _Q;
    R = _R;
    TS = ts;
    tolerance = 1e-6;
    max_num_iteration = 500;
}

void LQR::discrete(Eigen::MatrixXf &_A, Eigen::MatrixXf &_B, int type)
{
    A_d.resize(_A.rows(), _A.cols());
    B_d.resize(_B.rows(), _B.cols());
    Eigen::MatrixXf I(_A.rows(), _A.cols());
    I.setIdentity();
    if (type == 1) // front Euler
    {
        A_d = I + TS * _A;
        B_d = TS * _B;
    }
    else if (type == 2) // zero order
    {
        // todo
    }
    else
    {
        std::cout << "error type" << std::endl;
    }
}

/**
 * @brief
 *
 * @param A
 * @param B
 * @param is_dis    是否需要离散化
 *                  true    对输入矩阵进行离散
 *                  false   输入矩阵已经离散
 */
void LQR::compute_ARE(Eigen::MatrixXf &A, Eigen::MatrixXf &B, bool is_dis)
{
    Eigen::MatrixXf Ad_T, Bd_T;
    // check dim
    if ((A.rows() != A.cols()) && (A.rows() != B.rows()))
    {
        std::cerr << "mat A B dim diff\n";
        return;
    }
    // check controllable
    if (!is_controllable(A, B))
    {
        std::cerr << "system uncontrollable\n";
        return;
    }

    if (is_dis)
    {
        discrete(A, B, 1);
    }
    else
    {
        A_d.resize(A.rows(), A.cols());
        B_d.resize(B.rows(), B.cols());
        A_d = A;
        B_d = B;
    }
    Ad_T = A_d.transpose();
    Bd_T = B_d.transpose();

    // set P=Q;
    // P = Q;

    double diff = 0;

    auto t_start = std::chrono::steady_clock::now();

    for (uint i = 0; i < max_num_iteration; ++i)
    {
        P_next = Q + (Ad_T * P * A_d) - (Ad_T * P * B_d) * (R + Bd_T * P * B_d).inverse() * (Bd_T * P * A_d);
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        if (diff < tolerance)
        {
            K = (R + Bd_T * P * B_d).inverse() * (Bd_T * P * A_d);
            auto t_end = std::chrono::steady_clock::now();
            double t_cost_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
#ifdef LQR_LOG
            std::cout << "diff = " << diff << " cost time = " << t_cost_ms << std::endl
                      << "circle = " << i << std::endl;
            std::cout << "K = " << K << std::endl;
#endif
            return;
        }
    }
    std::cout << name << ": failed to solver riccati in max " << max_num_iteration << std::endl;
    K.resize(dim_u, dim_x);
    K.setOnes();
}

bool LQR::is_controllable(Eigen::MatrixXf &A, Eigen::MatrixXf &B)
{

    if ((A.rows() != A.cols()) && (A.rows() != B.rows()))
    {
        std::cerr << "mat A B dim diff\n";
        return false;
    }

    Eigen::MatrixXf tmp;
    tmp.setIdentity(A.rows(), A.rows());
    Eigen::MatrixXf mat_CTRL(B.rows(), B.rows());

    for (int i = 0; i < B.rows(); i++)
    {
        if (i != 0)
        {
            tmp = tmp * A;
        }

        mat_CTRL.col(i) = tmp * B;
    }

    if (mat_CTRL.determinant() == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}