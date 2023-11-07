/**
 * @file score.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief 计分用
 * @version 0.1
 * @date 2023-10-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "common_msgs/Control_Test.h"
#include "common_msgs/CICV_Location.h"
#include "common_msgs/Lane.h"
#include "common_msgs/Lanes.h"
#include "common_msgs/Perceptionobject.h"
#include "common_msgs/Perceptionobjects.h"

#include "control.h"
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

class car_obs_t : public control_t
{
public:
    const float sample_time = 0.04;
    float N, N1, N2, N3, N4, follow_coef;
    int ax_cnt, ay_cnt, jx_cnt, jy_cnt;
    int frame_sum, follow_frame_sum, not_idea_cnt;

    float comfortable_score, complete_score, safety_score, all_score;

    bool is_complete, compute_complete;

    car_obs_t();
    void sample_data();
    void compute_score();
};

car_obs_t::car_obs_t()
{
    is_complete = 1;
    compute_complete = false;
}

chrono::_V2::steady_clock::time_point start_time;

car_obs_t car_obs;
vector<float> visy, plotx0, plotx1, plotx2, plotx3;
long fig1 = plt::figure();

ros::Subscriber location_sub;
ros::Subscriber obstacle_sub;
ros::Subscriber lane_sub;
ros::Timer score;

void location_callback(const common_msgs::CICV_Location &msg);
void obstacle_callback(const common_msgs::Perceptionobjects &msg);
void lane_callback(const common_msgs::Lanes &msg);
void observe_callback(const ros::TimerEvent &e);

int main(int argc, char **argv)
{
    visy.resize(500);
    plotx0.resize(500);
    plotx1.resize(500);
    plotx2.resize(500);
    plotx3.resize(500);

    for (int i = 0; i < 500; i++)
    {
        visy[i] = (float)i * 0.1;
    }

    ros::init(argc, argv, "compute");
    ros::NodeHandle nh;

    location_sub = nh.subscribe("/cicv_location", 1000, location_callback);
    obstacle_sub = nh.subscribe("/tpperception", 1000, obstacle_callback);
    lane_sub = nh.subscribe("/LaneDetection", 1000, lane_callback);
    score = nh.createTimer(ros::Duration(car_obs.sample_time), observe_callback);

    start_time = std::chrono::steady_clock::now();

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
    car_obs.self.update(msg, car_obs.lane);
}

void obstacle_callback(const common_msgs::Perceptionobjects &msg)
{
    car_obs.obt.update(msg);
    car_obs.leader_update();
}

void lane_callback(const common_msgs::Lanes &msg)
{
    car_obs.lane.update(msg);
}

void observe_callback(const ros::TimerEvent &e)
{
    if (!car_obs.compute_complete)
    {
        car_obs.sample_data();
    }
    if (car_obs.self.is_in_destination())
    {
        car_obs.compute_score();
    }

    // 可视化当前车道线
    for (int i = 0; i < 500; i++)
    {
        plotx0[i] = -car_obs.lane.compute_lane_y(visy[i], 0);
        plotx1[i] = -car_obs.lane.compute_lane_y(visy[i], 1);
        plotx2[i] = -car_obs.lane.compute_lane_y(visy[i], 2);
        plotx3[i] = -car_obs.lane.compute_lane_y(visy[i], 3);
    }
    plt::clf();
    plt::plot(plotx0, visy);
    plt::plot(plotx1, visy);
    plt::plot(plotx2, visy);
    plt::plot(plotx3, visy);

    plt::xlim(-25, 25);
    plt::pause(0.01);
}

void car_obs_t::sample_data()
{
    static int follow_flag = 0;
    float dis_des = 5 + 1.5 * self.v_x;
    frame_sum++;
    if (self.a_x < -5 || self.a_x > 3)
    {
        ax_cnt++;
    }
    if (self.a_y < -1.5 || self.a_y > 1.5)
    {
        ay_cnt++;
    }
    if (self.j_x < -4 || self.j_x > 4)
    {
        jx_cnt++;
    }
    if (self.j_y < -2 || self.j_y > 2)
    {
        jy_cnt++;
    }

    if (self.start_follow)
    {
        follow_coef = (leader.line_len - dis_des) / dis_des;

        if (follow_flag == 0 && self.v_x >= 0.6 * leader.v_x && self.v_x <= 1.4 * leader.v_x)
        {
            follow_flag = 1;
        }
        if (follow_flag == 1)
        {
            follow_frame_sum++;
            if (follow_coef <= -0.5 || follow_coef >= 0.3)
            {
                not_idea_cnt++;
            }
            if (follow_coef <= -0.65 || follow_coef >= 0.6)
            {
                is_complete = 0;
            }
        }
    }

    std::cout << " ax_c: " << ax_cnt
              << " ay_c: " << ay_cnt
              << " jx_c: " << jx_cnt
              << " jy_c: " << jy_cnt
              << " ni_f: " << not_idea_cnt << std::endl;
}

void car_obs_t::compute_score()
{
    static int flag = 0;
    if (flag == 0)
    {
        float sum_seq = frame_sum;
        N1 = (float)(ay_cnt / sum_seq) * (float)100;
        N2 = (float)(jy_cnt / sum_seq) * (float)100;
        N3 = (float)(ax_cnt / sum_seq) * (float)100;
        N4 = (float)(jx_cnt / sum_seq) * (float)100;
        comfortable_score = 8 - N1 * 0.5 + 7 - N2 * 0.5 + 8 - N3 * 0.5 + 7 - N4 * 0.5;

        N = (float)not_idea_cnt / (float)follow_frame_sum * (float)100;
        safety_score = 40 - N;
        if (is_complete)
        {
            complete_score = 30;
        }

        all_score = comfortable_score + safety_score + complete_score;

        std::cout << " ax_c: " << ax_cnt
                  << " ay_c: " << ay_cnt
                  << " jx_c: " << jx_cnt
                  << " jy_c: " << jy_cnt
                  << " ni_f: " << not_idea_cnt << std::endl;

        std::cout << " sim_frame: " << sum_seq << " follow_frame: " << follow_frame_sum << std::endl;
        std::cout << " N: " << N << " N1: " << N1 << " N2: " << N2 << " N3: " << N3 << " N4: " << N4 << std::endl;
        std::cout << " cf_s: " << comfortable_score << " sf_s: " << safety_score << " cp_s: " << complete_score << std::endl;
        std::cout << " all score: " << all_score << std::endl;

        time_t now = time(0);
        std::ofstream score("/home/hx/Desktop/hx/code/ros/VTD_WS/src/test_ctrl/data/score" + std::to_string(now) + ".txt");
        if (score)
        {
            score << " ax_c: " << ax_cnt
                  << " ay_c: " << ay_cnt
                  << " jx_c: " << jx_cnt
                  << " jy_c: " << jy_cnt
                  << " ni_f: " << not_idea_cnt << std::endl
                  << " sim_frame: " << sum_seq << std::endl
                  << " N: " << N << " N1: " << N1 << " N2: " << N2 << " N3: " << N3 << " N4: " << N4 << std::endl
                  << " cf_s: " << comfortable_score << " sf_s: " << safety_score << " cp_s: " << complete_score << std::endl
                  << " all score: " << all_score << std::endl;
        }
        score.close();
        std::cout << "log flie saved" << std::endl;
        compute_complete = true;
        flag++;
    }
}