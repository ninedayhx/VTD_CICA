/**
 * @file MPC_pro_t.h
 * @author ninedayhx (1170535490@qq.com)
 * @brief 线性MPC基本程序
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <iostream>
#include <string>

#include "eigen3/Eigen/Eigen"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "discrete_lyapunov_equation.h"
#include "OsqpEigen/OsqpEigen.h"
#include "yaml-cpp/yaml.h"

using ESMd = Eigen::SparseMatrix<double>;
using EMXd = Eigen::MatrixXd;
using EVXd = Eigen::VectorXd;

class MPC_pro_t
{
public:
    int Nc, n, m;
    EMXd Ap, Bp;
    EMXd Qa, Ra;
    ESMd Hessian;
    EMXd F;
    EMXd K_mpc;
    EMXd L;

    OsqpEigen::Solver solver;
    MPC_pro_t(EMXd _A, EMXd _B, EMXd _Q, EMXd _R, int _Nc);
    std::tuple<EMXd, EMXd> PredictAB(EMXd _A, EMXd _B);
    std::tuple<EMXd, EMXd> AugmentQR(EMXd _Q, EMXd _R);
    EMXd AddTerminalCostP(EMXd _A, EMXd _B, EMXd _Q, EMXd _R);
    EMXd SolveNoConstraintMPCGain();
    void SetOsqpSolver(YAML::Node cfg);
    void InitHardConstraintsMatofSolver();
    void InitSoftConstraintsMatofSolver();
    bool InitOsqpSolver(ESMd _H, ESMd _L, EVXd _grad, EVXd _UB);
};