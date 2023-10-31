/**
 * @file LQR.h
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __LQR_H
#define __LQR_H

#include <chrono>
#include <string>
#include <iostream>
#include "eigen3/Eigen/Eigen"

// #define LQR_LOG

class LQR
{
private:
public:
    std::string name;
    float tolerance;
    int max_num_iteration;
    float TS;
    int dim_x, dim_u;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    Eigen::MatrixXf P;
    Eigen::MatrixXf P_next;
    Eigen::MatrixXf K;
    Eigen::MatrixXf A_d;
    Eigen::MatrixXf B_d;

    LQR(std::string _name);
    ~LQR();
    void get_param(Eigen::MatrixXf _Q, Eigen::MatrixXf _R, float ts);
    void discrete(Eigen::MatrixXf &_A, Eigen::MatrixXf &_B, int type);
    void compute_ARE(Eigen::MatrixXf &A, Eigen::MatrixXf &B, bool is_dis);
    bool is_controllable(Eigen::MatrixXf &A, Eigen::MatrixXf &B);
};

#endif
