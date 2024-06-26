/**
 * @file test_ctrl_node.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023
 *
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "yaml-cpp/yaml.h"

#include "control.h"

#define MAIN_LOG

using namespace std;

chrono::_V2::steady_clock::time_point start_time;

float mpc_filter, thr_filter;
int pub_log;
YAML::Node cfg;

control_t car_ctrl;
LQR LQR_lateral("LQR_lateral"), LQR_longtitute("LQR_longtitute"), LQR_lon_du("LQR_lon_du");
Eigen::MatrixXf Q_lat1(2, 2), R_lat1(1, 1);
Eigen::MatrixXf Q_lat2(2, 2), R_lat2(1, 1);
MPC_follow_t *mpc_lon, *mpc_sp;

common_msgs::Control_Test ctrl_msg;
ros::Subscriber location_sub;
ros::Subscriber obstacle_sub;
ros::Subscriber lane_sub;
ros::Publisher ctrl_pub;
ros::Publisher debug_pub;

ros::Timer controller;

void location_callback(const common_msgs::CICV_Location &msg);
void obstacle_callback(const common_msgs::Perceptionobjects &msg);
void lane_callback(const common_msgs::Lanes &msg);
void controller_callback(const ros::TimerEvent &e);
YAML::Node load_params(string path);

int main(int argc, char **argv)
{
    // yaml 需要使用空格缩进
    string cfg_path = ros::package::getPath("test_ctrl") + "/cfg/cfg.yaml";
    cfg = load_params(cfg_path);
    YAML::Node osqp_cfg = cfg["osqp"];

    mpc_filter = cfg["ctrl"]["mpc_filter"].as<float>();
    thr_filter = cfg["ctrl"]["thr_filter"].as<float>();
    car_ctrl.last_dis = cfg["ctrl"]["last_dis"].as<float>();
    car_ctrl.bias1 = cfg["ctrl"]["bias1"].as<float>();
    car_ctrl.bias2 = cfg["ctrl"]["bias2"].as<float>();
    car_ctrl.test_dis = cfg["ctrl"]["test_dis"].as<float>();
    pub_log = cfg["ctrl"]["pub_log"].as<int>();
    car_ctrl.sp_p = cfg["ctrl"]["sp_p"].as<float>();
    car_ctrl.sp_i = cfg["ctrl"]["sp_i"].as<float>();
    car_ctrl.sp_d = cfg["ctrl"]["sp_d"].as<float>();
    car_ctrl.use_soft_start = cfg["ctrl"]["use_soft_start"].as<int>();

    cout << "-----controller param-----" << endl;
    cout << "mpc filter: " << mpc_filter << endl;
    cout << "thr filter: " << thr_filter << endl;
    cout << "  last_dis: " << car_ctrl.last_dis << endl;
    cout << "     bias1: " << car_ctrl.bias1 << endl;
    cout << "     bias2: " << car_ctrl.bias2 << endl;
    cout << "      sp_p: " << car_ctrl.sp_p << endl;
    cout << "      sp_i: " << car_ctrl.sp_i << endl;
    cout << "      sp_d: " << car_ctrl.sp_d << endl;
    cout << "use_soft_start: " << car_ctrl.use_soft_start << endl;

    // clang-format off
    // Eigen::MatrixXf Q_lat1(2, 2), R_lat1(1, 1);
    Q_lat1 <<10, 0, 
             0,10;
    R_lat1 <<50000;
    // Eigen::MatrixXf Q_lat2(2, 2), R_lat2(1, 1);
    Q_lat2 << 10, 0, 
              0, 10;
    R_lat2 << 300;

    LQR_lateral.get_param(Q_lat1, R_lat1, 0.01);

    Eigen::MatrixXf Q_lon(3, 3), R_lon(1, 1);
    Q_lon << 60, 0,  0,
             0,  10, 0,
             0,  0,  1000;
    R_lon << 50;
    LQR_longtitute.get_param(Q_lon, R_lon, 0.01);
    
    Eigen::MatrixXf Q_lon_du(4, 4), R_lon_du(1, 1);

    Q_lon_du << 75,  0,  0,   0,
                0,   1,  0,   0,
                0,   0,  1,  0,
                0,   0,  0,   1000;
    R_lon_du << 500;
    LQR_lon_du.get_param(Q_lon_du, R_lon_du, 0.01);

    Eigen::MatrixXd Q_mpc(3,3),R_mpc(1,1),A_test(3,3),B_test(3,1);
    Q_mpc << 100,0,0,
             0,100,0,
             0,0,100;
    R_mpc << 5;
    // clang-format on

    const vector<double> QVec = cfg["mpc"]["Q"].as<vector<double>>();
    Q_mpc(0, 0) = QVec[0];
    Q_mpc(1, 1) = QVec[4];
    Q_mpc(2, 2) = QVec[8];
    R_mpc(0, 0) = cfg["mpc"]["R"].as<double>();
    Eigen::VectorXd Rho;
    Rho.resize(3);
    const vector<double> rhoVec = cfg["mpc"]["rho"].as<vector<double>>();
    for (int i = 0; i < 3; i++)
    {
        Rho(i) = rhoVec[i];
    }

    mpc_lon = new MPC_follow_t(car_ctrl.follow_leader_mod.A.cast<double>(),
                               car_ctrl.follow_leader_mod.B.cast<double>(),
                               Q_mpc,
                               R_mpc,
                               Rho,
                               cfg["mpc"]["Np"].as<int>(),
                               2,
                               3,
                               cfg);

    ros::init(argc, argv, "test_ctrl_node");
    ros::NodeHandle nh;

    location_sub = nh.subscribe("/cicv_location", 1000, location_callback);
    obstacle_sub = nh.subscribe("/tpperception", 1000, obstacle_callback);
    lane_sub = nh.subscribe("/LaneDetection", 1000, lane_callback);
    ctrl_pub = nh.advertise<common_msgs::Control_Test>("/control_test", 1000, false);
    debug_pub = nh.advertise<std_msgs::Float32MultiArray>("/debug_log", 1000, false);

    controller = nh.createTimer(ros::Duration(0.01), controller_callback);

    start_time = std::chrono::steady_clock::now();

    // LQR_lateral.compute_ARE(car_ctrl.sim_err_mod.A, car_ctrl.sim_err_mod.B, true);
    LQR_longtitute.compute_ARE(car_ctrl.follow_leader_mod.A, car_ctrl.follow_leader_mod.B, true);
    // LQR_lon_du.compute_ARE(car_ctrl.follow_du_mod.A, car_ctrl.follow_du_mod.B, false);

    // ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // ctrl_pub.publish(ctrl_msg);
        ros::spinOnce();
    }
    return 0;
}

void location_callback(const common_msgs::CICV_Location &msg)
{
    car_ctrl.self.update(msg, car_ctrl.lane);
}

void obstacle_callback(const common_msgs::Perceptionobjects &msg)
{
    car_ctrl.obt.update(msg);
    car_ctrl.leader_update();
}

void lane_callback(const common_msgs::Lanes &msg)
{
    car_ctrl.lane.update(msg);
}

void controller_callback(const ros::TimerEvent &e)
{
    static float a_tmp = 0;
    static int s_flag = 0;

    if (car_ctrl.self.p_x <= car_ctrl.test_dis && car_ctrl.car_cur.size() >= 2)
    {
        car_ctrl.self.start_follow = 0;
        car_ctrl.start_flag = 0;
    }
    if (car_ctrl.self.p_y > 45 && car_ctrl.self.phi > M_PI * 0.25 && car_ctrl.self.phi < M_PI * 0.75)
    {
        LQR_lateral.get_param(Q_lat2, R_lat2, 0.01);
        // cout << "force change" << endl;
    }
    LQR_lateral.compute_ARE(car_ctrl.sim_err_mod.A, car_ctrl.sim_err_mod.B, true);
    // cout << "k" << LQR_lateral.K << endl;

    if (car_ctrl.self.start_follow)
    {
        car_ctrl.update_state_vec();
        mpc_lon->compute_inequality_constraints(car_ctrl.x_k.cast<double>(), (double)car_ctrl.leader.v_x, true, (double)car_ctrl.self.a_x);

        if (!mpc_lon->solve_MPC_QP_with_constraints(car_ctrl.x_k.cast<double>(), LQR_longtitute.K.cast<double>(), true))
        {
            cout << "mpc solve fault" << endl;
        }
        ctrl_msg = car_ctrl.self.acc_to_Thr_and_Bra((float)mpc_lon->u_apply(0), mpc_filter, thr_filter);
    }
    else
    {
        if (car_ctrl.use_soft_start == 0)
        {
            if (car_ctrl.self.v_x > 0 && s_flag == 0)
            {
                // std::cout << "test5" << std::endl;
                car_ctrl.lon_v_des = car_ctrl.self.v_x * 3.6;
                // a_tmp = car_ctrl.self.a_x;
                s_flag = 1;
            }
            if (s_flag == 1)
            {
                float t_in = 0.01;
                a_tmp = a_tmp + 1.0 * t_in;
                if (a_tmp >= 3)
                {
                    // cout << "test" << endl;
                    a_tmp = 3;
                }
                car_ctrl.lon_v_des = car_ctrl.lon_v_des + mpsTokmph(a_tmp * t_in);
                if (car_ctrl.lon_v_des >= 30)
                {
                    // cout << "test" << endl;
                    car_ctrl.lon_v_des = 30;
                }
                ctrl_msg = car_ctrl.lon_speed_control(car_ctrl.lon_v_des);
            }
        }
        else
        {
            if (car_ctrl.self.v_x > 0 && car_ctrl.start_flag == 0)
            {
                std::cout << "test5" << std::endl;
                car_ctrl.lon_v_des = car_ctrl.self.v_x * 3.6;
                a_tmp = car_ctrl.self.a_x;
                car_ctrl.start_flag = 1;
            }
            if (car_ctrl.start_flag == 1)
            {
                float t_in = 0.01;
                a_tmp = a_tmp + 2.0 * t_in;
                if (a_tmp >= 3)
                {
                    // cout << "test" << endl;
                    a_tmp = 3;
                }
                car_ctrl.lon_v_des = car_ctrl.lon_v_des + mpsTokmph(a_tmp * t_in);
                if (car_ctrl.lon_v_des >= 30)
                {
                    // cout << "test" << endl;
                    car_ctrl.lon_v_des = 30;
                }
                // car_ctrl.lon_v_des = 30;
                ctrl_msg = car_ctrl.lon_speed_control(car_ctrl.lon_v_des);
            }
        }
    }

    // cout << "self p" << car_self.lane.lane_locate << "lead p " << car_self.leader.lane << endl;

    ctrl_msg.header.stamp = ros::Time::now();
    ctrl_msg.SteeringAngle = car_ctrl.lane_keep_LQR_control(LQR_lateral);
    ctrl_pub.publish(ctrl_msg);

    std_msgs::Float32MultiArray dmsg;

    if (pub_log == 1)
    {
        dmsg.data.resize(14);
        dmsg.data[0] = car_ctrl.lane.lane_phi;
        dmsg.data[1] = car_ctrl.lane.lane_center_err;
        dmsg.data[2] = car_ctrl.self.a_y;
        dmsg.data[3] = car_ctrl.self.j_y;

        dmsg.data[4] = car_ctrl.x_k(0);
        dmsg.data[5] = car_ctrl.x_k(1);
        dmsg.data[6] = car_ctrl.x_k(2);
        dmsg.data[7] = mpc_lon->u_apply(0);
        dmsg.data[8] = mpc_lon->du;
        dmsg.data[9] = car_ctrl.self.a_x;
        dmsg.data[10] = car_ctrl.self.j_x;
        debug_pub.publish(dmsg);
    }
}

YAML::Node load_params(string path)
{
    YAML::Node tmp;
    try
    {
        cout << "cfg path:" << path << endl;
        tmp = YAML::LoadFile(path);
        return tmp;
    }
    catch (YAML::ParserException)
    {
        cout << "config file no found, check first" << endl;
        tmp = YAML::Load("{0}");
        return tmp;
    }
}
