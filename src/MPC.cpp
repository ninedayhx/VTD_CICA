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
 * @param constraint_type   1 for hard constraint
 *                          2 for soft constraint
 * @param sc_num soft constraint num
 */
MPC_follow_t::MPC_follow_t(EMXd A, EMXd B, EMXd Q, EMXd R, int Np_, int constraint_type, int sc_num)
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

    epsilon.resize(3);

    H_s.resize(_H.rows() + sc_num, _H.cols() + sc_num);
    grad_s.resize(grad.rows() + sc_num);
    L_s.resize(L.rows() + sc_num, L.cols() + sc_num);
    LB_s.resize(LB.rows() + sc_num);
    UB_s.resize(UB.rows() + sc_num);

    if (constraint_type == 1)
    {
        solver_init(_H, grad, LB, UB, L, false);
    }
    else if (constraint_type == 2)
    {

        compute_Hessian_with_slack(sc_num);
        LB_s = -OsqpEigen::INFTY * LB_s.setOnes();
        compute_Linear_mat_with_slack(sc_num);

        if (!solver_init(H_s, grad_s, LB_s, UB_s, L_s, false))
        {
            std::cout << "osqp solver init false" << std::endl;
        }
        else
        {
            std::cout << "osqp solver init success" << std::endl;
        }
    }
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

bool MPC_t::solver_init(ESMd h, EVXd grad_, EVXd lb, EVXd ub, ESMd l, bool is_log)
{
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(is_log);
    // solver.settings()->setMaxIteration(10000);
    solver.settings()->setTimeLimit(0.005);
    solver.settings()->setAbsoluteTolerance(0.1);
    solver.settings()->setRelativeTolerance(0.1);
    solver.settings()->setPrimalInfeasibilityTolerance(0.01);
    solver.settings()->setDualInfeasibilityTolerance(0.01);
    solver.settings()->setDelta(0.001);

    if (solver.data()->isSet())
    {
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
    }

    solver.data()->setNumberOfVariables(h.cols());
    solver.data()->setNumberOfConstraints(l.rows());
    if (!solver.data()->setHessianMatrix(h))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(l))
        return false;
    if (!solver.data()->setLowerBound(lb))
        return false;
    if (!solver.data()->setGradient(grad_))
        return false;
    if (!solver.data()->setUpperBound(ub))
        return false;

    if (!solver.initSolver())
        return false;

    return true;
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

void MPC_t::compute_Hessian_with_slack(int sc_num)
{
    EMXd H_d, diag_rho; // dense mat of Hessian
    diag_rho.resize(sc_num, sc_num);
    for (int i = 0; i < sc_num; i++)
    {
        diag_rho(i, i) = rho[i];
    }
    H_d.resize(H_s.rows(), H_s.cols());
    H_d.setZero();
    H_d.block(0, 0, _H.rows(), _H.cols()) = _B.transpose() * _Q * _B + _R;
    H_d.block(_H.rows(), _H.cols(), sc_num, sc_num) = diag_rho;
    H_s = H_d.sparseView();
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
    grad_s.setZero();
    grad_s.block(0, 0, grad.rows(), grad.cols()) = grad;
}

void MPC_t::compute_Linear_mat_with_slack(int sc_num)
{
    EMXd tmp, ident, tmp_uf;
    tmp.resize(L.rows() + sc_num, L.cols() + sc_num);
    tmp.setZero();
    tmp.block(0, 0, L.rows(), L.cols()) = L.toDense();

    ident.setIdentity(sc_num, sc_num);

    EVXd one_2m(2 * m), one_2np(2 * Np);
    one_2m.setOnes();
    one_2np.setOnes();

    tmp_uf.resize(L.rows(), sc_num);
    tmp_uf.block(0, 0, 2 * m, 1) = -1.0 * one_2m;
    tmp_uf.block(2 * m, 1, 2 * m, 1) = -1.0 * one_2m;
    // 将u和du改为硬约束
    // tmp_uf.block(0, 0, 2 * m, 1).setZero();
    // tmp_uf.block(2 * m, 1, 2 * m, 1).setZero();
    tmp_uf.block(4 * m, 2, 2 * Np, 1) = -1.0 * one_2np;

    tmp.block(0, 0, L.rows(), L.cols()) = L.toDense();
    tmp.block(0, L.cols(), L.rows(), sc_num) = tmp_uf;
    tmp.block(L.rows(), L.cols(), sc_num, sc_num) = -1.0 * ident;

    L_s = tmp.sparseView();
}

bool MPC_t::solve_MPC_QP_no_constraints(EMXd x_k)
{
    return false;
}

/**
 * @brief 带线性不等式约束的qp求解，使用此api前
 *
 * @param x_k
 * @return true
 * @return false
 */
bool MPC_t::solve_MPC_QP_with_constraints(EMXd x_k, bool is_soft)
{
    compute_gradient(x_k);
    if (!is_soft)
    {
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
    }
    else
    {
        if (!solver.updateGradient(grad_s))
        {
            std::cout << "update grad err" << std::endl;
            return false;
        }
        if (!solver.updateUpperBound(UB_s))
        {
            std::cout << "update UB err" << std::endl;
            return false;
        }
    }

    OsqpEigen::ErrorExitFlag err_flag = solver.solveProblem();
    switch (err_flag)
    {
    case OsqpEigen::ErrorExitFlag::NoError:
        // std::cout << solver.workspace()->info->solve_time << std::endl;
        if (strcmp(solver.workspace()->info->status, "solved"))
        {
            std::cout << solver.workspace()->info->status << std::endl;
            std::cout << solver.workspace()->info->solve_time << std::endl;
        }
        else
        {
            std::cout << solver.workspace()->info->solve_time << std::endl;
        }
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
    epsilon = U_solve.block(U_solve.rows() - 3, 0, 3, 1);
    du = u_apply(0) - u_last(0);
    u_last = u_apply;

#ifdef MPC_LOG
    std::cout << "U_solve" << std::endl
              << U_solve << std::endl;
#endif

    return true;
}

void MPC_follow_t::compute_inequality_constraints(EVXd xk, double v, bool is_soft, double a_last)
{
    UB.resize(4 * Np * m + 2 * Np);

    EVXd one, u_tmp;
    one.resize(Np);
    u_tmp.resize(m);
    u_tmp(0) = a_last;
    one = one.setOnes();
    V_self.resize(Np);
    V_self = v * V_self.setOnes();

    UB << U_max,
        -U_min,
        // dU_max + W * u_tmp,
        // dU_max - W * u_tmp,
        dU_max + W * u_last,
        dU_max - W * u_last,
        1.5 * one + 0.45 * V_self - E * _A * xk,
        2.5 * one + 0.75 * V_self + E * _A * xk;

    UB_s.setZero();
    UB_s.block(0, 0, UB.rows(), UB.cols()) = UB;

    // std::cout
    //     << "UB:" << std::endl
    //     << U_max(0) << std::endl
    //     << -U_min(0) << std::endl
    //     << (dU_max + W * u_last)(0) << std::endl
    //     << (1.5 * one + 0.45 * V_self - E * _A * xk)(0) << std::endl;
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

void MPC_follow_t::add_soft_constraint()
{
}