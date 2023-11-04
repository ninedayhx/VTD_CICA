/**
 * @file MPC.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-11-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "MPC.h"

// /**
//  * @brief Construct a new mpc t::mpc t object
//  *
//  * @param A
//  * @param B
//  * @param Q
//  * @param R
//  * @param Np_
//  */
// // MPC_t::MPC_t(EMXd A, EMXd B, EMXd Q, EMXd R, int Np_)
// // {
//     // if (A.rows() != A.cols())
//     // {
//     //     std::cout << "A dim err" << std::endl;
//     //     return;
//     // }
//     // if (A.cols() != B.rows())
//     // {
//     //     std::cout << "B dim err" << std::endl;
//     //     return;
//     // }
//     // if (Q.rows() != Q.cols())
//     // {
//     //     std::cout << "Q dim err" << std::endl;
//     //     return;
//     // }
//     // if (n != Q.rows())
//     // {
//     //     std::cout << "cost Q mat dim err" << std::endl;
//     //     return;
//     // }
//     // if (R.rows() != R.cols())
//     // {
//     //     std::cout << "R dim err" << std::endl;
//     //     return;
//     // }
//     // if (m != R.rows())
//     // {
//     //     std::cout << "cost R mat dim err" << std::endl;
//     //     return;
//     // }
//     // if (Np_ <= 0)
//     // {
//     //     std::cout << "predict horizen err" << std::endl;
//     //     return;
//     // }

//     // n = A.rows();
//     // m = B.cols();
//     // Np = Np_;

//     // u_last.resize(m, 1);

//     // predict_AB(A, B);
//     // augmenting_Q(Q);
//     // augmenting_R(R);
//     // compute_Hessian();

//     // _A,_B,_Q,_R,_H,C are only determined by init value
//     // L determined by user
//     // grad determined by x(k|k)
// }

/**
 * @brief Construct a new mpc follow t::mpc follow t object
 *
 * @param A
 * @param B
 * @param Q
 * @param R
 * @param Np_
 */
MPC_follow_t::MPC_follow_t(EMXd A, EMXd B, EMXd Q, EMXd R, int Np_)
{
    Np = Np_;

    if (A.rows() != A.cols())
    {
        std::cout << "A dim err" << std::endl;
        return;
    }
    if (A.cols() != B.rows())
    {
        std::cout << "B dim err" << std::endl;
        return;
    }

    n = A.rows();
    m = B.cols();

    if (Q.rows() != Q.cols())
    {
        std::cout << "Q dim err" << std::endl;
        return;
    }
    if (n != Q.rows())
    {
        std::cout << "cost Q mat dim err" << std::endl;
        return;
    }
    if (R.rows() != R.cols())
    {
        std::cout << "R dim err" << std::endl;
        return;
    }
    if (m != R.rows())
    {
        std::cout << "cost R mat dim err" << std::endl;
        return;
    }
    if (Np_ <= 0)
    {
        std::cout << "predict horizen err" << std::endl;
        return;
    }

    std::cout << " MPC n: " << n << " m: " << m << " Np: " << Np << std::endl;

    _H.resize(m * Np, m * Np);
    grad.resize(m * Np);

    predict_AB(A, B);
    augmenting_Q(Q);
    augmenting_R(R);
    compute_Hessian();

    L.resize(4 * Np * m + 2 * Np, Np * m);

    EMXd L_tmp, I, G;
    I.setIdentity(Np * m, Np * m);
    G.setZero(Np * m, Np * m);
    G.block(m, 0, (Np - 1) * m, (Np - 1) * m).setIdentity((Np - 1) * m, (Np - 1) * m);

    E.resize(Np, Np * n);
    E.setZero();
    for (int i = 0; i < Np; i++)
    {
        E(i, i * n) = 1;
    }

    L_tmp.resize(4 * Np * m + 2 * Np, Np * m);
    L_tmp << I, -I, I - G, G - I, E * _B, -E * _B;
    L = L_tmp.sparseView();

    // constrict
    LB.resize(4 * Np * m + 2 * Np);
    LB = -OsqpEigen::INFTY * LB.setOnes();

    UB.resize(4 * Np * m + 2 * Np);

    U_max.resize(Np * m);
    U_max = u_max * U_max.setOnes();

    U_min.resize(Np * m);
    U_min = u_min * U_min.setOnes();

    dU_max.resize(Np * m);
    dU_max = du_max * dU_max.setOnes();

    W.resize(Np * m, m);
    W.block(0, 0, m, m).setIdentity(m, m);

    u_last.resize(m);
    u_apply.resize(m);
    U_solve.resize(m * Np);

    // 求解器设置
    solver.settings()->setWarmStart(true);
    // solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(_H.cols());
    solver.data()->setNumberOfConstraints(L.rows());
    if (!solver.data()->setHessianMatrix(_H))
        return;
    if (!solver.data()->setLinearConstraintsMatrix(L))
        return;
    if (!solver.data()->setLowerBound(LB))
        return;
    if (!solver.data()->setGradient(grad))
        return;
    if (!solver.data()->setUpperBound(UB))
        return;

    if (!solver.initSolver())
        return;

#ifdef MPC_LOG
    std::cout << "_A" << std::endl
              << _A << std::endl
              << "_B" << std::endl
              << _B << std::endl
              << "_Q" << std::endl
              << _Q << std::endl
              << "_R" << std::endl
              << _R << std::endl
              << "_H" << std::endl
              << _H << std::endl;
#endif

    // _A,_B,_Q,_R,_H,C are only determined by init value
    // D determined by u_last
    // grad determined by x(k|k)

    // init
}

void MPC_t::discrete(EMXd A, EMXd B, int type)
{
    A_d.resize(A.rows(), A.cols());
    B_d.resize(B.rows(), B.cols());
    Eigen::MatrixXd I(A.rows(), A.cols());
    I.setIdentity();
    if (type == 1) // front Euler
    {
        A_d = I + TS * A;
        B_d = TS * B;
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

void MPC_t::predict_AB(EMXd A, EMXd B)
{
    discrete(A, B, 1);

    _A.resize(n * Np, n);
    for (int i = 0; i < Np; i++)
    {
        _A.block(i * n, 0, n, n) = A_d.pow(i + 1);
    }

    _B.resize(n * Np, m * Np);
    for (int i = 0; i < Np; i++)
    {
        for (int j = 0; j < Np; j++)
        {
            if (j <= i)
            {
                _B.block(i * n, j * m, n, m) = A_d.pow(i - j) * B_d;
            }
            else
            {
                _B.block(i * n, j * m, n, m) = Eigen::MatrixXd::Zero(n, m);
            }
        }
    }
}

void MPC_t::augmenting_Q(EMXd Q)
{

    _Q.resize(n * Np, n * Np);
    for (int i = 0; i < Np; i++)
    {
        _Q.block(i * n, i * n, n, n) = Q;
    }
}

void MPC_t::augmenting_R(EMXd R)
{
    _R.resize(m * Np, m * Np);
    for (int i = 0; i < Np; i++)
    {
        _R.block(i * m, i * m, m, m) = R;
    }
}

void MPC_t::compute_Hessian()
{
    EMXd H_d; // dense mat of Hessian
    H_d = _B.transpose() * _Q * _B + _R;
    _H = H_d.sparseView();
}

void MPC_t::compute_gradient(EMXd x_k)
{
    if (x_k.rows() != n)
    {
        std::cout << "state vec dim err" << std::endl;
        return;
    }
    _F = _B.transpose() * _Q * _A;
    // std::cout << " _F " << std::endl
    //   << _F << std::endl;
    grad = _F * x_k;
}

bool MPC_t::solve_MPC_QP_no_constraints(EMXd x_k)
{
}

/**
 * @brief 带线性不等式约束的qp求解，使用此api前
 *
 * @param x_k
 * @return true
 * @return false
 */
bool MPC_t::solve_MPC_QP_with_constraints(EMXd x_k)
{
    compute_gradient(x_k);

    if (!solver.updateGradient(grad))
    {
        std::cout << "update grad err" << std::endl;
        return false;
    }
    if (!solver.updateUpperBound(UB))
    {
        std::cout << "update UB err" << std::endl;
        return false;
    }

    OsqpEigen::ErrorExitFlag err_flag = solver.solveProblem();
    switch (err_flag)
    {
    case OsqpEigen::ErrorExitFlag::NoError:
        // std::cout << "solver ok" << std::endl;
        break;
    case OsqpEigen::ErrorExitFlag::DataValidationError:
        std::cout << "DataValidationError" << std::endl;
        return false;
        break;
    case OsqpEigen::ErrorExitFlag::LinsysSolverInitError:
        std::cout << "LinsysSolverInitError" << std::endl;
        return false;
        break;
    case OsqpEigen::ErrorExitFlag::LinsysSolverLoadError:
        std::cout << "LinsysSolverLoadError" << std::endl;
        return false;
        break;
    case OsqpEigen::ErrorExitFlag::NonCvxError:
        std::cout << "NonCvxError" << std::endl;
        return false;
        break;
    case OsqpEigen::ErrorExitFlag::SettingsValidationError:
        std::cout << "SettingsValidationError" << std::endl;
        return false;
        break;
    case OsqpEigen::ErrorExitFlag::WorkspaceNotInitError:
        std::cout << "WorkspaceNotInitError" << std::endl;
        return false;
        break;
    default:
        std::cout << "unknown err" << std::endl;
        return false;
        break;
    }

    U_solve = solver.getSolution();
    u_apply = U_solve.block(0, 0, m, 1);
    u_last = u_apply;

#ifdef MPC_LOG
    std::cout << "U_solve" << std::endl
              << U_solve << std::endl;
#endif

    return true;
}

void MPC_follow_t::compute_inequality_constraints(EVXd xk, double v)
{
    UB.resize(4 * Np * m + 2 * Np);

    EVXd one;
    one.resize(Np);
    one = one.setOnes();
    V_self.resize(Np);
    V_self = v * V_self.setOnes();

    UB << U_max,
        -U_min,
        dU_max + W * u_last,
        dU_max - W * u_last,
        1.5 * one + 0.45 * V_self - E * _A * xk,
        2.5 * one + 0.75 * V_self + E * _A * xk;

#ifdef MPC_LOG
    std::cout
        << "U_max:" << std::endl
        << U_max << std::endl
        << "U_min:" << std::endl
        << U_min << std::endl
        << "dU_max:" << std::endl
        << dU_max << std::endl
        << "W:" << std::endl
        << W << std::endl
        << "u_last:" << std::endl
        << u_last << std::endl;
    std::cout << "UB:" << std::endl
              << UB << std::endl;
#endif
}