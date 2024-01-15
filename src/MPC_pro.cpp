/**
 * @file MPC_pro.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "MPC_pro.h"
#include "LQR.h"

MPC_pro_t::MPC_pro_t(EMXd _A, EMXd _B, EMXd _Q, EMXd _R, int _Nc)
{
    Nc = _Nc;
    n = _A.rows();
    m = _B.cols();

    // predict
    auto [Ap, Bp] = PredictAB(_A, _B);
    auto [Qa, Ra] = AugmentQR(_Q, _R);

    // add terminal cost
    Qa.block((Nc - 1) * n, (Nc - 1) * n, n, n) = AddTerminalCostP(_A, _B, _Q, _R);

    // convert to standard QP problem
    Hessian = (Bp.transpose() * Qa * Bp + Ra).sparseView();

    F = Bp.transpose() * Qa * Ap;
}

std::tuple<EMXd, EMXd> MPC_pro_t::PredictAB(EMXd _A, EMXd _B)
{

    EMXd _Ap(n * Nc, n), _Bp(n * Nc, m * Nc);

    for (int i = 0; i < Nc; i++)
    {
        _Ap.block(i * n, 0, n, n) = _A.pow(i + 1);
    }

    for (int i = 0; i < Nc; i++)
    {
        for (int j = 0; j < Nc; j++)
        {
            if (j <= i)
            {
                _Bp.block(i * n, j * m, n, m) = _A.pow(i - j) * _B;
            }
            else
            {
                _Bp.block(i * n, j * m, n, m) = Eigen::MatrixXd::Zero(n, m);
            }
        }
    }

    Ap = _Ap;
    Bp = _Bp;

    return {_Ap, _Bp};
}

std::tuple<EMXd, EMXd> MPC_pro_t::AugmentQR(EMXd _Q, EMXd _R)
{

    EMXd _Qa(n * Nc, n * Nc), _Ra(m * Nc, m * Nc);

    for (int i = 0; i < Nc; i++)
    {
        _Qa.block(i * n, i * n, n, n) = _Q;
    }

    for (int i = 0; i < Nc; i++)
    {
        _Ra.block(i * m, i * m, m, m) = _R;
    }

    Qa = _Qa;
    Ra = _Ra;

    return {_Qa, _Ra};
}

EMXd MPC_pro_t::AddTerminalCostP(EMXd _A, EMXd _B, EMXd _Q, EMXd _R)
{

    EMXd Ak, Qk, K, P;

    // 使用LQR计算最优K
    LQR dlqr("dlqr");
    dlqr.get_param(_Q, _R, 0.01);
    dlqr.compute_ARE(_A, _B, false);
    K = dlqr.K;

    Ak = _A - _B * K;
    Qk = _Q + K.transpose() * _R * K;

    P = drake::math::RealDiscreteLyapunovEquation(Ak, Qk);

    return P;
}

EMXd MPC_pro_t::SolveNoConstraintMPCGain()
{
    K_mpc = ((Bp.transpose() * Qa * Bp + Ra).transpose() * Bp.transpose() * Qa * Ap).block(0, 0, 1, n);
    return K_mpc;
}

void MPC_pro_t::SetOsqpSolver(YAML::Node cfg)
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
}

void MPC_pro_t::InitHardConstraintsMatofSolver()
{
}
void MPC_pro_t::InitSoftConstraintsMatofSolver()
{
}

bool MPC_pro_t::InitOsqpSolver(ESMd _H, ESMd _L, EVXd _grad, EVXd _UB)
{
    EVXd LB;
    LB.resize(_UB.rows());
    LB = -OsqpEigen::INFTY * LB.setOnes();

    if (solver.data()->isSet())
    {
        solver.data()->clearHessianMatrix();
        solver.data()->clearLinearConstraintsMatrix();
    }

    solver.data()->setNumberOfVariables(_H.rows());
    solver.data()->setNumberOfConstraints(_L.rows());
    if (!solver.data()->setHessianMatrix(_H))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(_L))
        return false;
    if (!solver.data()->setLowerBound(LB))
        return false;
    if (!solver.data()->setGradient(_grad))
        return false;
    if (!solver.data()->setUpperBound(_UB))
        return false;

    if (!solver.initSolver())
        return false;

    return true;
}
