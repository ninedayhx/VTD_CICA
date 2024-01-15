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
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd K;
    Eigen::MatrixXd A_d;
    Eigen::MatrixXd B_d;

    LQR(std::string _name);
    ~LQR();
    void get_param(Eigen::MatrixXd _Q, Eigen::MatrixXd _R, float ts);
    void discrete(Eigen::MatrixXd &_A, Eigen::MatrixXd &_B, int type);
    void compute_ARE(Eigen::MatrixXd &A, Eigen::MatrixXd &B, bool is_dis);
    bool is_controllable(Eigen::MatrixXd &A, Eigen::MatrixXd &B);
};

#endif
