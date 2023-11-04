/**
 * @file control.c
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-10-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "control.h"

float radTodeg(float rad)
{
    float deg = rad / M_PI * 180.0f;
    return deg;
}
float degTorad(float deg)
{
    float rad = deg / 180.0f * M_PI;
    return rad;
}
/***********************************************/
/*                   构造函数                   */
/***********************************************/
control_t::control_t()
{
    sim_err_mod.A.resize(2, 2);
    sim_err_mod.B.resize(2, 1);
    sim_err_mod.A << 0, 0, 30, 0;
    sim_err_mod.B << 30 / 2.7, 0;

    // clang-format off
    float T_delay = 0.01;
    follow_leader_mod.A.resize(3, 3);
    follow_leader_mod.B.resize(3, 1);
    follow_leader_mod.A <<  0, 1, 0, 
                            0, 0, -1, 
                            0, 0, -1 / T_delay;
    follow_leader_mod.B << 0, 0, 1 / T_delay;

    float T_d = 0.1;
    float T_s = 0.01;
    follow_du_mod.A.resize(4, 4);
    follow_du_mod.B.resize(4, 1);
    follow_du_mod.A <<  1,  T_s,  0,              0,
                        0,  1,   -T_s,            0,
                        0,  0,    1 - T_s / T_d,  T_s / T_d,
                        0,  0,    0,              1;
    follow_du_mod.B <<  0, 
                        0, 
                        T_s / T_d, 
                        1;
    // clang-format on
    x_k.resize(3);

    self.L_des = 0;
    leader.line_len = 0;
}

/***********************************************/
/*                   car_t                     */
/***********************************************/
bool car_t::is_stopped()
{
    if (v_x <= 0.1 && v_y <= 0.01)
    {
        return true;
    }
    else
    {
        return false;
    }
}
/***********************************************/
/*                  leader_t                   */
/***********************************************/
void leader_t::update(common_msgs::Perceptionobject msg, lane_param _lane, car_self _self)
{
    phi = msg.heading;
    d_phi = phi - _self.phi;
    p_x = msg.xg;
    p_y = msg.yg;
    d_x = msg.x;
    d_y = msg.y;
    v_x = msg.v_xg * cos(phi) + msg.v_yg * sin(phi);
    v_y = -msg.v_xg * cos(phi) + msg.v_yg * sin(phi);

    a_x_last = a_x;
    a_y_last = a_y;
    a_x = (msg.a_x + _self.a_x) * cos(d_phi) + (msg.a_y + _self.a_y) * sin(d_phi);
    a_y = (msg.a_x + _self.a_x) * sin(d_phi) + (msg.a_y + _self.a_y) * cos(d_phi);

    j_x = (a_x - a_x_last) * data_hz;
    j_y = (a_y - a_y_last) * data_hz;

    distance = pow(msg.x * msg.x + msg.y * msg.y, 0.5) - _lane.car_front_len; // 当跟车距离很近时，近似为绝对距离
    line_len = _lane.compute_line_length(d_x, _lane.car_front_len);
}

/***********************************************/
/*                  car_self                   */
/***********************************************/
car_self::car_self()
{
}
void car_self::update(const common_msgs::CICV_Location &msg, lane_param _lane)
{
    phi = msg.Yaw;

    p_x = msg.Position_x;
    p_y = msg.Position_y;
    v_x = msg.Velocity_x * cos(phi) + msg.Velocity_y * sin(phi);
    v_y = -msg.Velocity_x * sin(phi) + msg.Velocity_y * cos(phi);

    a_x_last = a_x;
    a_y_last = a_y;
    a_x = msg.Accel_x * cos(phi) + msg.Accel_y * sin(phi);
    a_y = -msg.Accel_x * sin(phi) + msg.Accel_y * cos(phi);

    j_x = (a_x - a_x_last) * data_hz;
    j_y = (a_y - a_y_last) * data_hz;

    lane = _lane.lane_locate;
}

common_msgs::Control_Test car_self::acc_to_Thr_and_Bra(float a_des, bool en_filter)
{
    float u;
    float alpha = 0.1;
    static float a_last = 0;

    if (en_filter)
    {
        a_des = alpha * a_des + (1 - alpha) * a_last;
    }
    a_last = a_des;

    if (a_des >= 0)
    {
        if (v_x < 5 && v_x >= 0)
        {
            u = a_des / 5.4;
        }
        else
        {
            u = p[0] + p[1] * v_x + p[2] * a_des +
                p[3] * v_x * v_x + p[4] * v_x * a_des + p[5] * a_des * a_des;
        }
    }
    if (a_des < 0)
    {
        if (v_x >= 0.01)
        {
            u = a_des / (float)10;
        }
        else
        {
            u = 0;
        }
    }
    u_des = a_des;

    common_msgs::Control_Test msg;
    if (u >= 0)
    {
        msg.Gear = 4;          // 1-P 2-R 3-N 4-D
        msg.BrakePedal = 0;    // 0-1
        msg.ThrottlePedal = u; // 0-1
    }
    else
    {
        msg.Gear = 4;
        msg.BrakePedal = -u;
        msg.ThrottlePedal = 0;
    }
    return msg;
}

bool car_self::is_in_destination()
{
    if ((p_y < -22) && (phi > M_PI))
    {
        if (is_stopped())
        {
            return true;
        }
    }
    return false;
}

/***********************************************/
/*                  obtacle                    */
/***********************************************/
void obtacle::update(const common_msgs::Perceptionobjects &msg)
{
    data_raw = msg;
}

/***********************************************/
/*                  lane_param                 */
/***********************************************/
void lane_param::update(const common_msgs::Lanes &msg)
{
    int tmp, tmp_cnt = 0;

    for (int i = 0; i < msg.num; i++)
    {
        tmp = (int)msg.lanes[i].lane_idx;
        lane[tmp].id = tmp;
        lane[tmp].c0 = msg.lanes[i].c_0;
        lane[tmp].c1 = msg.lanes[i].c_1;
        lane[tmp].c2 = msg.lanes[i].c_2;
        lane[tmp].c3 = msg.lanes[i].c_3;
        tmp_cnt += tmp;
    }
    // std::cout << "l0: " << lane[0].id << " l1: " << lane[1].id << " l2: " << lane[2].id << " l3: " << lane[3].id << std::endl;
    if (tmp_cnt == 3) // right
    {
        lane[3].c0 = 0;
        lane[3].c1 = 0;
        lane[3].c2 = 0;
        lane[3].c3 = 0;
        lane[3].id = 0;
        lane_locate = 2;
        lane_center_err = (lane[1].c0 - lane[2].c0) / 2 - lane[1].c0;
    }
    else if (tmp_cnt == 6) // left
    {
        lane[0].c0 = 0;
        lane[0].c1 = 0;
        lane[0].c2 = 0;
        lane[0].c3 = 0;
        lane[0].id = 0;
        lane_locate = 1;
        lane_center_err = (lane[1].c0 - lane[2].c0) / 2 - lane[1].c0;
    }
    else
    {
        std::cout << "lane_err" << tmp_cnt << std::endl;
    }
    lane_phi = -atan(lane[1].c1);                       // 后轴与车道线夹角
    lane_phi_forward = -2 * lane[1].c2 * car_front_len; // 车头与车道线夹角

#ifdef LANE_LOG
    for (int j = 0; j < 4; j++)
    {
        std::cout << "L:" << (int)lane[j].id << std::endl
                  << " c0:" << lane[j].c0 << " c1" << lane[j].c1 << " c2:" << lane[j].c2 << " c3:" << lane[j].c3 << std::endl;
    }
#endif
}

float lane_param::compute_lane_y(float x, int id)
{
    return lane[id].c0 + lane[id].c1 * x + lane[id].c2 * x * x + lane[id].c3 * pow(x, 3);
}

float lane_param::compute_lane_rel_angle(float dx, int id)
{
    return lane[id].c1 + 2 * lane[id].c2 * dx + 3 * lane[id].c3 * dx * dx;
}

double lane_param::compute_line_length(float x2, float x1)
{
    // L=\int_x_1^x_2 \squa ()
    double len = int_compute(x2, x1, 10);
    return len;
}

double lane_param::int_compute(float x2, float x1, int n)
{
    double df = 0;
    float c[4];
    c[0] = lane[1].c0;
    c[1] = lane[1].c1;
    c[2] = lane[1].c2;
    c[3] = lane[1].c3;

    double step = (double)(x2 - x1) / (double)n;
    for (int i = 0; i < n; i++)
    {
        df = df + line_func(c, x1 + (double)i * step) * step;
    }
    return df;
}

double lane_param::line_func(float *c, float x)
{
    double res = pow((double)1 + pow((double)c[1] + (double)2 * c[2] * x + (double)3 * c[3] * x * x, 2), 0.5);
    return res;
}

/***********************************************/
/*                  control_t                  */
/***********************************************/

void control_t::leader_update()
{
    std::vector<common_msgs::Perceptionobject> car;
    for (int i = 0; i < obt.data_raw.num; i++)
    {
        if (obt.data_raw.Perceptionobjects[i].type == 1) // car
        {
            car.push_back(obt.data_raw.Perceptionobjects[i]);
        }
    }

    int car_num = car.size();
    if (car_num == 0)
    {
        std::cout << "no leader car" << std::endl;
    }
    if (car_num == 1)
    {
        leader.update(car[0], lane, self);
        is_lane_changing();
    }
    if (car_num > 1)
    {
        std::cout << "car num err" << std::endl;
        // todo
    }
}

void control_t::is_lane_changing()
{
    float lane_y = 0;
    float lane_leader_abs_ang = atan(lane.compute_lane_rel_angle(leader.d_x, 1)) + self.phi; // rad
    float lane_leader_rel_ang = leader.phi - lane_leader_abs_ang;                            // rad

    if (lane.lane_locate == 1)
    {
        lane_y = lane.compute_lane_y(leader.d_x, 2);
        if (leader.d_y >= lane_y)
        {
            self.start_follow = 1;
            leader.lane = 1;
        }
        else
        {
            // 通过前车和车道线的夹角，预先跟踪前车
            if (radTodeg(lane_leader_rel_ang) > 3) // deg
            {
                self.start_follow = 1;
                leader.lane = 2;
                // std::cout << "rel_ang" << radTodeg(lane_leader_rel_ang) << std::endl;
            }
            else if (radTodeg(lane_leader_rel_ang) <= 3)
            {
                self.start_follow = 0;
                leader.lane = 2;
            }
        }
    }
    else if (lane.lane_locate == 2)
    {
        lane_y = lane.compute_lane_y(leader.d_x, 1);
        // std::cout << "leader y: " << car[0].y << " lane y: " << lane_y << std::endl;
        if (leader.d_y <= lane_y)
        {
            self.start_follow = 1;
            leader.lane = 2;
        }
        else
        {
            if (radTodeg(lane_leader_rel_ang) < -3) // deg
            {
                self.start_follow = 1;
                leader.lane = 1;
            }
            else if (radTodeg(lane_leader_rel_ang) >= -3)
            {
                self.start_follow = 0;
                leader.lane = 1;
            }
        }
    }
    else
    {
        std::cout << "leader lane err" << std::endl;
    }
}

/**
 * @brief
 *
 * @param speed_des
 * @return float
 */
common_msgs::Control_Test control_t::lon_speed_control(float speed_des)
{
    static float err2 = 0;
    static float err3 = 0;
    static float u_last = 0;
    float kp, ki, kd, des, err, du, u;
    kp = 1;
    ki = 0;
    kd = 0.1;

    err = speed_des * 1000 / 60 / 60 - self.v_x;

    du = kp * (err - err2) + ki * err + kd * (err - 2 * err2 + err3);

    u = du + u_last;

    err3 = err2;
    err2 = err;
    u_last = u;

    if (u > 2.9)
    {
        u = 2.9;
    }

    return self.acc_to_Thr_and_Bra(u, true);
}

/**
 * @brief 基于简单误差模型的LQR车道保持
 *
 * @param lane
 * @param _lqr
 * @return float
 */
float control_t::lane_keep_LQR_control(LQR _lqr)
{
    float err_phi, err_dis;
    err_phi = lane.lane_phi;
    err_dis = lane.lane_center_err;

    float u_delta = -_lqr.K(0, 0) * err_phi - _lqr.K(0, 1) * err_dis;
    float u_delta_forward = lane.lane_phi_forward;
    float u_angle = u_delta - self.v_x / self.l_fr * lane.lane_phi_forward;
    u_angle = radTodeg(u_delta); // rad to deg

    // 输出限制
    if (u_angle > self.max_delta)
    {
        u_angle = self.max_delta;
    }
    else if (u_delta < -self.max_delta)
    {
        u_angle = -self.max_delta;
    }

    float steer_angle = u_angle / self.max_delta * self.max_steer_angle;
#ifdef CAR_LOG
    std::cout << "u_delta_angle " << u_delta / M_PI * 180 << std::endl;
    std::cout << "u_delta_forward " << u_delta_forward / M_PI * 180 << std::endl;
    std::cout << "angle " << steer_angle << std::endl;
#endif

    return steer_angle;
}

/**
 * @brief
 *
 * @param _lqr
 * @param leader
 * @return float 车辆期望加速度
 */
float control_t::leader_follow_LQR_control(LQR _lqr)
{
    static int follow_flag = 0;

    // self.L_des = 5 + 1.5 * self.v_x;

    // float dx = leader.line_len - self.L_des;
    // float dv = leader.v_x - self.v_x;
    // float da = leader.a_x - self.a_x;

    float a_des = -(_lqr.K(0, 0) * x_k(0) + _lqr.K(0, 1) * x_k(1) + _lqr.K(0, 2) * x_k(2));
    // float a_des = -(_lqr.K(0, 0) * dx + _lqr.K(0, 1) * dv + _lqr.K(0, 2) * da);

    if (a_des >= 5.4)
    {
        a_des = 5.4;
    }
    if (a_des < -4.9)
    {
        a_des = -4.9;
    }

    lon_a_des = a_des;
    return a_des;
}

float control_t::leader_follow_LQR_du_control(LQR _lqr)
{
    static int follow_flag = 0;
    static float a_des_last = 0;
    static float da_last, ax_last, lax_last;
    float a_des, ax_filter, lax_filter;

    // self.L_des = 5 + 1.5 * self.v_x;

    // float dx = leader.line_len - self.L_des;
    // float dv = leader.v_x - self.v_x;
    // float da = leader.a_x - self.a_x;

    da_last = x_k(2);

    float da_des = -(_lqr.K(0, 0) * x_k(0) + _lqr.K(0, 1) * x_k(1) + _lqr.K(0, 2) * x_k(2) + _lqr.K(0, 3) * a_des_last);
    if (std::abs(da_des) > 5)
    {
        std::cout << "du" << da_des << std::endl;
    }
    a_des = da_des + a_des_last;
    a_des_last = a_des;
    if (a_des >= 5.4)
    {
        a_des = 5.4;
    }
    if (a_des < -4.9)
    {
        a_des = -4.9;
    }

    lon_a_des = a_des;
    return a_des;
}

void control_t::update_state_vec()
{
    self.L_des = 5 + 1.5 * self.v_x;

    x_k(0) = leader.line_len - self.L_des;
    x_k(1) = leader.v_x - self.v_x;
    x_k(2) = leader.a_x - self.a_x;
}
