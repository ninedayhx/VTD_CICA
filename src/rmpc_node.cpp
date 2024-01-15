/**
 * @file rmpc.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief 测试rmpc效果
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "yaml-cpp/yaml.h"

#include "control.h"
#include "MPC_pro.h"

using namespace std;

YAML::Node cfg;

control_t car_ctrl;
LQR LQR_lateral("LQR_lateral");
Eigen::MatrixXd Q_lat1(2, 2), R_lat1(1, 1);

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
    // string cfg_path = ros::package::getPath("test_ctrl") + "/cfg/rmpc.yaml";
    // cfg = load_params(cfg_path);
    // YAML::Node osqp_cfg = cfg["osqp"];

    // clang-format off
    // Eigen::MatrixXf Q_lat1(2, 2), R_lat1(1, 1);
    Q_lat1 <<10, 0, 
             0,10;
    R_lat1 <<300;

    LQR_lateral.get_param(Q_lat1, R_lat1, 0.01);

    // clang-format on
    EMXd mpc_Q(3, 3), mpc_R(1, 1);
    mpc_Q << 500, 0, 0,
        0, 500, 0,
        0, 0, 500;
    mpc_R << 10;

    MPC_pro_t test(car_ctrl.follow_leader_mod.A,
                   car_ctrl.follow_leader_mod.B,
                   mpc_Q,
                   mpc_R,
                   80);
    test.SolveNoConstraintMPCGain();

    ros::init(argc, argv, "test_ctrl_node");
    ros::NodeHandle nh;

    location_sub = nh.subscribe("/cicv_location", 1000, location_callback);
    obstacle_sub = nh.subscribe("/tpperception", 1000, obstacle_callback);
    lane_sub = nh.subscribe("/LaneDetection", 1000, lane_callback);
    ctrl_pub = nh.advertise<common_msgs::Control_Test>("/control_test", 1000, false);
    debug_pub = nh.advertise<std_msgs::Float32MultiArray>("/debug_log", 1000, false);

    controller = nh.createTimer(ros::Duration(0.01), controller_callback);

    LQR_lateral.compute_ARE(car_ctrl.sim_err_mod.A, car_ctrl.sim_err_mod.B, true);

    while (ros::ok())
    {
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
