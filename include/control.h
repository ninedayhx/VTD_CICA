/**
 * @file control.h
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CONTROL_H
#define __CONTROL_H

#include <fstream>
#include <vector>
#include <ctime>
#include <algorithm>

#include "common_msgs/Control_Test.h"
#include "common_msgs/CICV_Location.h"
#include "common_msgs/Lane.h"
#include "common_msgs/Lanes.h"
#include "common_msgs/Perceptionobject.h"
#include "common_msgs/Perceptionobjects.h"

#include "LQR.h"
#include "MPC.h"

float radTodeg(float rad);
float degTorad(float deg);
float kmphTomps(float rad);
float mpsTokmph(float rad);

/**
 * @brief 车辆原始信息类
 *
 */
class car_t
{
public:
    const int data_hz = 50;

    float phi;      // 车身转角 世界坐标系
    float p_x;      // 车身在世界坐标系下的位置
    float p_y;      // 车身在世界坐标系下的位置
    float v_x;      // 车身坐标系数据
    float v_y;      //
    float a_x;      //
    float a_y;      //
    float a_x_last; //
    float a_y_last; //
    float j_x;      //
    float j_y;      //
    int lane;       // 处于哪个车道 1 left 2 right

    bool is_stopped();
};

class LTI
{
public:
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;
};

/**
 * @brief   障碍物信息获取
 *
 */
class obtacle
{
private:
    /* data */
public:
    common_msgs::Perceptionobjects data_raw;
    std::vector<common_msgs::Perceptionobject> car;
    void update(const common_msgs::Perceptionobjects &msg);
};

/**
 * @brief
 *
 */
class lane_param
{
private:
public:
    const float car_front_len = 3.647; // 后轴到车头距离 m

    struct lane_t
    {
        int id;
        float c0;
        float c1;
        float c2;
        float c3;
    } lane[4];
    int lane_locate; // 1 left  2 right
    float lane_center_err;
    float lane_phi;         // 车道线相较于车身坐标系的夹角 rad
    float lane_phi_forward; // 前馈转角
    // lane_param();
    // ~lane_param();

    void update(const common_msgs::Lanes &msg);
    float compute_lane_y(float x, int id);
    float compute_lane_rel_angle(float dx, int id);

    double compute_line_length(float x1, float x2);
    double line_func(float *c, float x);
    double int_compute(float x2, float x1, int n);
};

/**
 * @brief 自车信息
 *
 */
class car_self : public car_t
{
public:
    const float car_front_len = 3.647; // 车辆后轴到前保险杠距离
    const float max_delta = 15;        // 最大转向角
    const float max_steer_angle = 540; // 最大方向盘转角
    const float l_fr = 2.691;          // 轴距
    const double p[6] = {0.01883,      //
                         -0.0002959,   //
                         -0.005006,    //
                         0.0003868,    //
                         0.03745,      //
                         -0.0004034};  // 加速度-油门的对应系数
                                       // const double q[3] = {0.02279,
                                       //                      0.0002106,
                                       //                      0.1819};
    const double q[3] = {0.01891,
                         0.0002017,
                         0.1829};
    float L_des; // 跟车间距
    float a_des_f;

    // state
    int start_follow;
    float u_des;

    car_self();
    void update(const common_msgs::CICV_Location &msg, lane_param _lane);
    common_msgs::Control_Test acc_to_Thr_and_Bra(float a, float a_filter, float u_filter);
    bool is_in_destination();
};

/**
 * @brief 前车信息
 *
 */
class leader_t : public car_t
{
public:
    float d_x;      // leader车相较于自车坐标系下的x位置
    float d_y;      // leader车相较于自车坐标系下的y位置
    float d_phi;    // leader方向和自车方向夹角
    float distance; // leader和follwer的距离,绝对值
    float line_len; // 沿轨迹距离

    void update(common_msgs::Perceptionobject msg, lane_param _lane, car_self _self);
};

/**
 * @brief
 *
 */
class control_t
{
public:
    const float search_dis = 40;
    float last_dis = 25;
    float bias1 = 0;
    float bias2 = 0;

    car_self self;
    leader_t leader;
    std::vector<common_msgs::Perceptionobject> car_cur, car_oth;
    obtacle obt;
    lane_param lane;

    LTI sim_err_mod;
    LTI follow_leader_mod;
    LTI follow_du_mod;

    Eigen::VectorXf x_k;

    float lon_v_des;
    float lon_a_des;

    control_t();

    void leader_update();
    void is_lane_changing();
    int is_lane_changing(std::vector<common_msgs::Perceptionobject> _car);

    int is_cutinto(common_msgs::Perceptionobject _car);
    void is_cutout();
    int find_the_latest(std::vector<common_msgs::Perceptionobject> _car);
    std::vector<common_msgs::Perceptionobject> find_current_lane_car(std::vector<common_msgs::Perceptionobject> _car);
    std::vector<common_msgs::Perceptionobject> find_other_lane_car(std::vector<common_msgs::Perceptionobject> _car);

    void update_state_vec();

    common_msgs::Control_Test lon_speed_control(float speed_des);
    float lane_keep_LQR_control(LQR _lqr);
    float leader_follow_LQR_control(LQR _lqr);
    float leader_follow_LQR_du_control(LQR _lqr);
};

#endif