/**
 * @file MPC.h
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __MPC_H
#define __MPC_H

#include <iostream>

#include "eigen3/Eigen/Eigen"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "OsqpEigen/OsqpEigen.h"

// #define MPC_LOG

using ESMd = Eigen::SparseMatrix<double>;
using EMXd = Eigen::MatrixXd;
using EVXd = Eigen::VectorXd;

class MPC_t
{
public:
    int Np;                   // 预测或控制时域 Np = Nc
    int Nc;                   // unuse
    int n;                    // 状态变量维度
    int m;                    // 控制变量维度
                              //
    EMXd A_d, B_d;            // 离散后的矩阵
    double TS = 0.01;         // 控制周期
                              //
    EMXd _A;                  // 预测状态矩阵 (n*Np)*n
    EMXd _B;                  // 预测控制矩阵 (n*Np)*(n*Np)
    EMXd _Q;                  // 增广Q矩阵 (n*Np)*(n*Np)
    EMXd _R;                  // 增广R矩阵 (m*Np)*(m*Np)
    EMXd _F;                  // 线性规划矩阵 _F = _B^T*_Q*_A (n*Np)*n
    ESMd _H;                  // hessian矩阵 _H = _B^T*_Q*_B+_R (m*Np)*(m*Np)
    EVXd grad;                // 梯度向量 grad = _F*x(k|k) (n*Np)*1
                              //
    EVXd u_last;              // 上次控制量
    EVXd U_solve;             // osqp求解出的Np步的控制量向量
    EVXd u_apply;             // 取当前时刻控制量
                              //
    ESMd L;                   // 线性约束矩阵 稀疏
    EVXd LB, UB;              // 上下限约束向量
                              //
    OsqpEigen::Solver solver; //

    void discrete(EMXd A, EMXd B, int type);
    void predict_AB(EMXd A, EMXd B);
    void augmenting_Q(EMXd Q);
    void augmenting_R(EMXd R);
    void compute_Hessian();
    void compute_gradient(EMXd x_k);
    bool solve_MPC_QP_no_constraints(EMXd x_k);
    bool solve_MPC_QP_with_constraints(EMXd x_k);

    // MPC_t(EMXd A, EMXd B, EMXd Q, EMXd R, int Np_);
};

class MPC_follow_t : public MPC_t
{
public:
    const double du_max = 0.04; // u最大增量
    const double u_max = 3;     // u上界约束
    const double u_min = -5;    // u下界约束
    EVXd U_max, U_min, dU_max;

    EMXd W;

    MPC_follow_t(EMXd A, EMXd B, EMXd Q, EMXd R, int Np_);
    void compute_inequality_constraints();
};

#endif