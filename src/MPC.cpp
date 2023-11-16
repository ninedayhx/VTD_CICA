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
 * @param constraint_type   0 for no constraint
 *                          1 for hard constraint
 *                          2 for soft constraint
 * @param sc_num            slack var num
 */
MPC_follow_t::MPC_follow_t(EMXd A, EMXd B, EMXd Q, EMXd R, EVXd _rho, int Np_, int constraint_type, int sc_num, YAML::Node cfg)
{
    Np = Np_;
    YAML::Node osqp_cfg, mpc_cfg;
    std::vector<int> soft_flag;
    mpc_cfg = cfg["mpc"];
    osqp_cfg = cfg["osqp"];

    fc_lb = mpc_cfg["fc_lb"].as<double>();
    fc_ub = mpc_cfg["fc_ub"].as<double>();
    soft_flag = mpc_cfg["soft_flag"].as<std::vector<int>>();
    u_max = mpc_cfg["u_max"].as<double>();
    u_min = mpc_cfg["u_min"].as<double>();
    du_max = mpc_cfg["du_max"].as<double>();
    use_lqr = mpc_cfg["use_lqr"].as<int>();

    rho.resize(sc_num);
    for (int i = 0; i < sc_num; i++)
    {
        rho(i) = _rho(i) * (double)soft_flag[i];
    }
    epsilon.resize(3);

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

    std::cout << "-----MPC param------" << std::endl;
    std::cout << " MPC n: " << n << " m: " << m << " Np: " << Np << std::endl;
    std::cout << " Q: " << std::endl
              << Q << std::endl;
    std::cout << " R: " << R << std::endl;
    std::cout << " Rho: " << rho.transpose() << std::endl;
    std::cout << " u_max: " << u_max << " u_min: " << u_min << " du_max: " << du_max << std::endl;

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

    H_s.resize(_H.rows() + sc_num, _H.cols() + sc_num);
    grad_s.resize(grad.rows() + sc_num);
    L_s.resize(L.rows() + sc_num, L.cols() + sc_num);
    LB_s.resize(LB.rows() + sc_num);
    UB_s.resize(UB.rows() + sc_num);

    if (constraint_type == 0)
    {
        solver_init(_H, grad, false);
    }
    else if (constraint_type == 1)
    {
        solver_init(_H, grad, LB, UB, L, cfg, false);
    }
    else if (constraint_type == 2)
    {
        compute_Linear_mat_with_slack(sc_num, soft_flag[0], soft_flag[1], soft_flag[2]);
        compute_Hessian_with_slack(sc_num);
        LB_s = -OsqpEigen::INFTY * LB_s.setOnes();
        if (!solver_init(H_s, grad_s, LB_s, UB_s, L_s, osqp_cfg, false))
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

bool MPC_t::solver_init(ESMd h, EVXd grad_, bool is_log)
{
    // clang-format off

    solver.settings()->setLinearSystemSolver(QDLDL_SOLVER);
    solver.settings()->setVerbosity(is_log);
    solver.settings()->setWarmStart(false);
    solver.settings()->setScaling(10);
    solver.settings()->setPolish(false);
    solver.settings()->setRho(0.1);

    solver.settings()->setSigma(1e-06);
    solver.settings()->setAlpha(1.6);



    solver.settings()->setAdaptiveRho(true);
    solver.settings()->setAdaptiveRhoInterval(0);
    solver.settings()->setAdaptiveRhoFraction(0.4);
    solver.settings()->setAdaptiveRhoTolerance(5);
    solver.settings()->setMaxIteration(4000);
    solver.settings()->setAbsoluteTolerance(0.001);
    solver.settings()->setRelativeTolerance(0.001);
    solver.settings()->setPrimalInfeasibilityTolerance(0.0001);
    solver.settings()->setDualInfeasibilityTolerance(0.0001);
    solver.settings()->setScaledTerimination(false);
    solver.settings()->setCheckTermination(25);
    solver.settings()->setTimeLimit(1e+10);
    solver.settings()->setPolishRefineIter(3);
    // clang-format on

    if (solver.data()->isSet())
    {
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
    }

    solver.data()->setNumberOfVariables(h.cols());
    solver.data()->setNumberOfConstraints(0);
    if (!solver.data()->setHessianMatrix(h))
        return false;
    if (!solver.data()->setGradient(grad_))
        return false;

    if (!solver.initSolver())
        return false;

    return true;
}

bool MPC_t::solver_init(ESMd h, EVXd grad_, EVXd lb, EVXd ub, ESMd l, YAML::Node cfg, bool is_log)
{
    assert(cfg.IsMap());
    if (cfg["default"].as<int>() == 1)
    {
        solver.settings()->setVerbosity(cfg["log"].as<int>());
        solver.settings()->setWarmStart(cfg["warmstart"].as<int>());
    }
    else
    {
        // clang-format off
        solver.settings()->setLinearSystemSolver(cfg["solver"].as<int>());
        solver.settings()->setVerbosity(cfg["log"].as<int>());
        solver.settings()->setWarmStart(cfg["warmstart"].as<int>());
        solver.settings()->setScaling(cfg["scaling"].as<int>());
        solver.settings()->setPolish(cfg["polish"].as<int>());
        solver.settings()->setRho(cfg["rho"].as<double>());

        solver.settings()->setSigma(cfg["sigma"].as<double>());
        solver.settings()->setAlpha(cfg["alpha"].as<double>());



        solver.settings()->setAdaptiveRho(cfg["adarho"].as<int>());
        solver.settings()->setAdaptiveRhoInterval(cfg["adarhoint"].as<int>());
        solver.settings()->setAdaptiveRhoFraction(cfg["adarhofrac"].as<double>());
        solver.settings()->setAdaptiveRhoTolerance(cfg["adarhotol"].as<double>());
        solver.settings()->setMaxIteration(cfg["miter"].as<int>());
        solver.settings()->setAbsoluteTolerance(cfg["atol"].as<double>());
        solver.settings()->setRelativeTolerance(cfg["rtol"].as<double>());
        solver.settings()->setPrimalInfeasibilityTolerance(cfg["pitol"].as<double>());
        solver.settings()->setDualInfeasibilityTolerance(cfg["ditol"].as<double>());
        solver.settings()->setScaledTerimination(cfg["scalter"].as<int>());
        solver.settings()->setCheckTermination(cfg["checkter"].as<int>());
        solver.settings()->setTimeLimit(cfg["timelim"].as<double>());
        solver.settings()->setPolishRefineIter(cfg["priter"].as<int>());
        // clang-format on
    }

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
        diag_rho(i, i) = rho(i);
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

void MPC_t::compute_Linear_mat_with_slack(int sc_num, int is_u_sf, int is_du_sf, int is_fc_sf)
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

    if (is_u_sf == 1)
    {
        tmp_uf.block(0, 0, 2 * m, 1) = -1.0 * one_2m;
    }
    else
    {
        tmp_uf.block(0, 0, 2 * m, 1).setZero();
    }

    if (is_du_sf == 1)
    {
        tmp_uf.block(2 * m, 1, 2 * m, 1) = -1.0 * one_2m;
    }
    else
    {
        tmp_uf.block(2 * m, 1, 2 * m, 1).setZero();
    }

    if (is_fc_sf == 1)
    {
        tmp_uf.block(4 * m, 2, 2 * Np, 1) = -1.0 * one_2np;
    }
    else
    {
        tmp_uf.block(4 * m, 2, 2 * Np, 1).setZero();
    }

    tmp.block(0, 0, L.rows(), L.cols()) = L.toDense();
    tmp.block(0, L.cols(), L.rows(), sc_num) = tmp_uf;
    tmp.block(L.rows(), L.cols(), sc_num, sc_num) = -1.0 * ident;

    L_s = tmp.sparseView();
}

bool MPC_t::solve_MPC_QP_no_constraints(EMXd x_k)
{
    compute_gradient(x_k);

    if (!solver.updateGradient(grad))
    {
        std::cout << "update grad err" << std::endl;
        return false;
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
    // std::cout << "U_solve" << std::endl
    //           << U_solve.size() << std::endl;
    // if (U_solve.size() >= m * (Np + 3))
    // {
    u_apply = U_solve.block(0, 0, m, 1);
    epsilon = U_solve.block(U_solve.rows() - 3, 0, 3, 1);
    du = u_apply(0) - u_last(0);
    u_last = u_apply;
    // }
    return true;
}

/**
 * @brief 带线性不等式约束的qp求解，使用此api前
 *
 * @param x_k
 * @return true
 * @return false
 */
bool MPC_t::solve_MPC_QP_with_constraints(EMXd x_k, EMXd lqr_k, bool is_soft)
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
            // U_solve = solver.getSolution();
            // u_apply = U_solve.block(0, 0, m, 1);
            // epsilon = U_solve.block(U_solve.rows() - 3, 0, 3, 1);
            // du = u_apply(0) - u_last(0);
            // u_last = u_apply;
            if (use_lqr)
            {
                u_apply(0) = -(lqr_k(0, 0) * x_k(0) + lqr_k(0, 1) * x_k(1) + lqr_k(0, 2) * x_k(2));
                u_last = u_apply;
            }
        }
        else
        {
            std::cout << solver.workspace()->info->solve_time << std::endl;
            U_solve = solver.getSolution();
            u_apply = U_solve.block(0, 0, m, 1);
            epsilon = U_solve.block(U_solve.rows() - 3, 0, 3, 1);
            du = u_apply(0) - u_last(0);
            u_last = u_apply;
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
    // U_solve = solver.getSolution();
    // // std::cout << "U_solve" << std::endl
    // //           << U_solve.size() << std::endl;
    // // if (U_solve.size() >= m * (Np + 3))
    // // {
    // u_apply = U_solve.block(0, 0, m, 1);
    // epsilon = U_solve.block(U_solve.rows() - 3, 0, 3, 1);
    // du = u_apply(0) - u_last(0);
    // u_last = u_apply;
    // }

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
    V_l.resize(Np);
    V_l = v * V_l.setOnes();

    UB << U_max,
        -U_min,
        // dU_max + W * u_tmp,
        // dU_max - W * u_tmp,
        dU_max + W * u_last,
        dU_max - W * u_last,
        // -0.5-0.3
        // 1.5 * one + 0.45 * V_l - E * _A * xk,
        // 2.5 * one + 0.75 * V_l + E * _A * xk;
        // -0.7-0.9
        fc_ub * 5.0 * one + fc_ub * 1.5 * V_l - E * _A * xk,
        fc_lb * 5.0 * one + fc_lb * 1.5 * V_l + E * _A * xk;

    UB_s.setZero();
    UB_s.block(0, 0, UB.rows(), UB.cols()) = UB;

    // std::cout
    //     << "UB:" << std::endl
    //     << U_max(0) << std::endl
    //     << -U_min(0) << std::endl
    //     << (dU_max + W * u_last)(0) << std::endl
    //     << (1.5 * one + 0.45 * V_l - E * _A * xk)(0) << std::endl;
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