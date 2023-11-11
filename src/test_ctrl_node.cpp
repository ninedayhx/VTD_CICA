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
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "control.h"

#define MAIN_LOG

using namespace std;

chrono::_V2::steady_clock::time_point start_time;

control_t car_ctrl;
LQR LQR_lateral("LQR_lateral"), LQR_longtitute("LQR_longtitute"), LQR_lon_du("LQR_lon_du");
MPC_follow_t *mpc_lon;

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

int main(int argc, char **argv)
{

    // clang-format off
    Eigen::MatrixXf Q_lat(2, 2), R_lat(1, 1);
    Q_lat << 10, 0, 
             0, 10;
    R_lat << 60;

    LQR_lateral.get_param(Q_lat, R_lat, 0.01);

    Eigen::MatrixXf Q_lon(3, 3), R_lon(1, 1);
    Q_lon << 60, 0,  0,
             0,  10, 0,
             0,  0,  1000;
    R_lon << 100;
    LQR_longtitute.get_param(Q_lon, R_lon, 0.01);

    Eigen::MatrixXf Q_lon_du(4, 4), R_lon_du(1, 1);

    Q_lon_du << 75,  0,  0,   0,
                0,   1, 0,   0,
                0,   0,  1,  0,
                0,   0,  0,   1000;
    R_lon_du << 500;
    LQR_lon_du.get_param(Q_lon_du, R_lon_du, 0.01);

    Eigen::MatrixXd Q_mpc(3,3),R_mpc(1,1),A_test(3,3),B_test(3,1);
    Q_mpc << 100,0,0,
             0,100,0,
             0,0,100;
    R_mpc << 5;
    // A_test = 2*A_test.setIdentity(3,3);
    // B_test.setOnes();
    // clang-format on

    mpc_lon = new MPC_follow_t(car_ctrl.follow_leader_mod.A.cast<double>(),
                               car_ctrl.follow_leader_mod.B.cast<double>(),
                               Q_mpc,
                               R_mpc,
                               80, 2, 3);

    ros::init(argc, argv, "test_ctrl_node");
    ros::NodeHandle nh;

    location_sub = nh.subscribe("/cicv_location", 1000, location_callback);
    obstacle_sub = nh.subscribe("/tpperception", 1000, obstacle_callback);
    lane_sub = nh.subscribe("/LaneDetection", 1000, lane_callback);
    ctrl_pub = nh.advertise<common_msgs::Control_Test>("/control_test", 1000, false);
    debug_pub = nh.advertise<std_msgs::Float32MultiArray>("/debug_log", 1000, false);

    controller = nh.createTimer(ros::Duration(0.01), controller_callback);

    start_time = std::chrono::steady_clock::now();

    LQR_lateral.compute_ARE(car_ctrl.sim_err_mod.A, car_ctrl.sim_err_mod.B, true);
    // LQR_longtitute.compute_ARE(car_ctrl.follow_leader_mod.A, car_ctrl.follow_leader_mod.B, true);
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
    static float flag = 0;

    if (car_ctrl.self.start_follow)
    {
        car_ctrl.update_state_vec();
        mpc_lon->compute_inequality_constraints(car_ctrl.x_k.cast<double>(), (double)car_ctrl.self.v_x, true);

        if (!mpc_lon->solve_MPC_QP_with_constraints(car_ctrl.x_k.cast<double>(), true))
        {
            cout << "mpc solve fault" << endl;
        }
        ctrl_msg = car_ctrl.self.acc_to_Thr_and_Bra((float)mpc_lon->u_apply(0), 0.01);

        // ctrl_msg = car_ctrl.self.acc_to_Thr_and_Bra(car_ctrl.leader_follow_LQR_du_control(LQR_lon_du), true);
        // ctrl_msg = car_ctrl.self.acc_to_Thr_and_Bra(car_ctrl.leader_follow_LQR_control(LQR_longtitute), 0.1);

        // cout << "following...  self lane: " << car_ctrl.lane.lane_locate << " leader lane: " << car_ctrl.leader.lane << endl;
    }
    else
    {
        float t_in = 0;
        if (flag != 0)
        {
            t_in = e.current_real.toSec() - e.last_real.toSec();
        }
        else
        {
            t_in = 0.01;
        }
        a_tmp = a_tmp + 2 * t_in;
        if (a_tmp >= 3)
        {
            cout << "test" << endl;
            a_tmp = 3;
        }
        // cout << "a_tmp" << a_tmp << "v_des" << car_ctrl.lon_v_des / 3.6 << endl;
        car_ctrl.lon_v_des = car_ctrl.lon_v_des + mpsTokmph(a_tmp * t_in);
        if (car_ctrl.lon_v_des >= 30)
        {
            cout << "test" << endl;

            car_ctrl.lon_v_des = 30;
        }
        cout << " a_tmp " << a_tmp << " v_des " << car_ctrl.lon_v_des << " t " << t_in << endl;

        ctrl_msg = car_ctrl.lon_speed_control(car_ctrl.lon_v_des);
        // cout << "no leader, self speed... lane = " << endl;
    }

    // cout << "self p" << car_self.lane.lane_locate << "lead p " << car_self.leader.lane << endl;

    ctrl_msg.header.stamp = ros::Time::now();
    ctrl_msg.SteeringAngle = car_ctrl.lane_keep_LQR_control(LQR_lateral);
    ctrl_pub.publish(ctrl_msg);

    std_msgs::Float32MultiArray dmsg;
    dmsg.data.resize(14);
    dmsg.data[0] = car_ctrl.x_k(0);
    dmsg.data[1] = car_ctrl.x_k(1);
    dmsg.data[2] = car_ctrl.x_k(2);
    dmsg.data[3] = (float)mpc_lon->u_apply(0);
    dmsg.data[4] = car_ctrl.self.a_des_f;
    dmsg.data[5] = car_ctrl.self.u_des;
    dmsg.data[6] = car_ctrl.self.a_x;
    dmsg.data[7] = car_ctrl.self.j_x;
    dmsg.data[8] = car_ctrl.leader.a_x;
    dmsg.data[9] = car_ctrl.self.a_x;
    dmsg.data[10] = mpc_lon->du;
    dmsg.data[11] = kmphTomps(car_ctrl.lon_v_des);
    dmsg.data[12] = car_ctrl.self.v_x;
    dmsg.data[13] = car_ctrl.lon_a_des;

    debug_pub.publish(dmsg);

    if (flag == 0)
    {
        flag = 1;
    }
}
