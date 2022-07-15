#include "nubot/nubot_control/behaviour.hpp"
using namespace nubot;

Behaviour::Behaviour() : radius_robot(50.0), radius_Obs(50.0)
{
    app_vx_ = 0;
    app_vy_ = 0;
    app_w_ = 0;
    isTurn_ = false;
    last_app_vx_ = 0;
    last_app_vy_ = 0;
    last_app_w_ = 0;
    action = nullptr;
}

Behaviour::~Behaviour()
{
}

void Behaviour::_min(int n, double *nums, int &index, double &val)
{
    index = 0;
    val = nums[0];
    for (int i = 1; i < n; ++i)
    {
        if (nums[i] < val)
        {
            index = i;
            val = nums[i];
        }
    }
}

void Behaviour::_max(int n, double *nums, int &index, double &val)
{
    index = 0;
    val = nums[0];
    for (int i = 1; i < n; ++i)
    {
        if (nums[i] > val)
        {
            index = i;
            val = nums[i];
        }
    }
}

void Behaviour::relocate(int obs_num, double *cos_cast, double *sin_cast,
                         int *obs_group, std::vector<DPoint> &Obstacles_, DPoint &r2t)
{
    int bp_num(0), bn_num(0);
    int left = 0, right = 0, sign_side = 0;
    double b_positive[10] = {0};
    double b_negative[10] = {0};
    for (int i = 0; i <= obs_num; i++)
        if (sin_cast[obs_group[i]] > 0)
            b_positive[bp_num++] = sin_cast[obs_group[i]];
        else if (sin_cast[obs_group[i]] < 0)
            b_negative[bn_num++] = fabs(sin_cast[obs_group[i]]);
    if (*std::max_element(b_positive, b_positive + 10) <= *std::max_element(b_negative, b_negative + 10))
    {
        left = 1;
        sign_side = 1;
    }
    else
    {
        right = 1;
        sign_side = -1;
    }

    double atemp = 0;
    bool canpass = 0;
    double alpha[9] = {0};
    double alpha_val = 0;
    int alpha_id = 0;

    for (int i = 0; i <= obs_num; i++)
    {
        atemp = Obstacles_.at(obs_group[i]).distance(robot_pos_);
        if (atemp < (radius_robot + radius_Obs))
        {
            atemp = radius_robot + radius_Obs + 0.0001;
            // canpass 可去
            canpass = 1;
        }
        alpha[i] = atan2(sin_cast[obs_group[i]], cos_cast[obs_group[i]]) + sign_side * asin((radius_robot + radius_Obs) / atemp);
    }
    if (left == 1)
        _max(obs_num + 1, alpha, alpha_id, alpha_val);
    else
        _min(obs_num + 1, alpha, alpha_id, alpha_val);

    m_subtarget.x_ = robot_pos_.x_ + (cos(alpha_val) * r2t.x_ -
                                      sin(alpha_val) * r2t.y_) *
                                         Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / r2t.length();
    m_subtarget.y_ = robot_pos_.y_ +
                     (sin(alpha_val) * r2t.x_ + cos(alpha_val) * r2t.y_) * Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / r2t.length();
}

bool Behaviour::movewithallObs(DPoint target, DPoint pos, double distance_thres, int k)
{
    subtarget(target, robot_pos_, true);
    target = m_subtarget;
    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = k * target.distance(pos);

    if (pos.distance(target) > distance_thres)
        return false;
    else
        return true;
}

void Behaviour::subtarget(DPoint target_pos_, DPoint robot_pos_, bool avoid_ball) //计算临时目标点
{
    std::vector<DPoint> Obstacles_ = this->Obstacles_;

    if (avoid_ball)
    {
        Obstacles_.push_back(ball_pos_);
    }

    for (int c = Obstacles_.size(); c < 10; c++)
        Obstacles_.push_back(DPoint(10000, 10000));

    double radius_robot = 50.0, radius_Obs = 50.0;
    double a[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, b[10];
    DPoint point_ = target_pos_ - robot_pos_; // Target point vector
    int i = 0, j = 0, k = 0;
    int B[10] = {0};
    int First_num = 0;
    double minB = 0;

    int G[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    int G_num = 0;
    int G_Obstacles_[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    double b_positive[11] = {0};
    double b_negative[11] = {0};
    int left = 0, right = 0, sign_side = 0, bp_num = 0, bn_num = 0;

    double atemp = 0;
    double alpha[10] = {0};
    double alpha_i = 0;
    int alpha_k = 0;

    bool canpass = 0;

    for (i = 0; i < 10; i++)
    {
        int temp = 0;
        DPoint point1_ = Obstacles_.at(i) - robot_pos_;
        if (point_.cross(point1_) == 0)
            temp = 0;
        else if (point_.cross(point1_) > 0)
            temp = 1;
        else
            temp = -1;

        a[i] = point_.ddot(point1_) / point_.length(); // Point multiplication for projection
        b[i] = temp * fabs(point_.cross(point1_)) / point_.length();
    }
    // obtain B that may hit
    for (i = 0; i < 10; i++)
    {
        if ((a[i] > 0) && (a[i] < point_.length()) && (fabs(b[i]) < (radius_robot + radius_Obs)))
        {
            B[j] = i;
            j++;
        }
    }

    if (j != 0)
    {
        // determine first Obstacles_
        First_num = B[0];
        minB = a[B[0]];
        for (i = 1; i < j; i++)
        {
            if (minB < a[B[i]])
                minB = minB;
            else
            {
                minB = a[B[i]];
                First_num = B[i];
            }
        }

        // Grouping--the Obstacles_ that must be avoided
        G[0] = First_num;
        G_num = 0;
        G_Obstacles_[First_num] = 0;

        for (i = 0; i < 10; i++)
        {
            if (G_Obstacles_[i] == 1)
            {
                for (k = 0; k <= G_num; k++)
                {
                    if (Obstacles_.at(i).distance(Obstacles_.at(G[k])) < (2 * radius_robot + 2 * radius_Obs))
                    {
                        G_num++;
                        G[G_num] = i;
                        G_Obstacles_[i] = 0;
                        i = -1;
                        break;
                    }
                }
            }
        }
        // Location of subtarget
        for (i = 0; i <= G_num; i++)
        {
            if (b[G[i]] > 0)
            {
                bp_num++;
                b_positive[bp_num] = b[G[i]];
            }
            else if (b[G[i]] < 0)
            {
                bn_num++;
                b_negative[bn_num] = fabs(b[G[i]]);
            }
        }
        if (*std::max_element(b_positive, b_positive + 11) <= *std::max_element(b_negative, b_negative + 11))
        {
            left = 1;
            sign_side = 1;
        }
        else
        {
            right = 1;
            sign_side = -1;
        }

        for (i = 0; i <= G_num; i++)
        {
            atemp = Obstacles_.at(G[i]).distance(robot_pos_);
            if (atemp < (radius_robot + radius_Obs))
            {
                atemp = radius_robot + radius_Obs + 0.0001;
                canpass = 1;
            }
            alpha[i] = atan2(b[G[i]], a[G[i]]) + sign_side * asin((radius_robot + radius_Obs) / atemp);
        }

        if (left == 1)
        {
            _max(G_num + 1, alpha, alpha_k, alpha_i);
        }
        else
        {
            _min(G_num + 1, alpha, alpha_k, alpha_i);
        }

        m_subtarget.x_ = robot_pos_.x_ + (cos(alpha_i) * point_.x_ - sin(alpha_i) * point_.y_) * Obstacles_.at(G[alpha_k]).distance(robot_pos_) / point_.length();
        m_subtarget.y_ = robot_pos_.y_ + (sin(alpha_i) * point_.x_ + cos(alpha_i) * point_.y_) * Obstacles_.at(G[alpha_k]).distance(robot_pos_) / point_.length();
    }
    else
        m_subtarget = target_pos_;
}

void Behaviour::subtarget(DPoint &target_pos_, DPoint &robot_pos_)
{
    std::vector<DPoint> Obstacles_ = this->Obstacles_;

    for (int c = Obstacles_.size(); c < 9; c++)
        Obstacles_.push_back(DPoint(10000, 10000));
    double cos_cast[9], sin_cast[9];
    DPoint r2t = target_pos_ - robot_pos_;
    int i = 0, j = 0, k = 0;
    int B[9] = {0};
    int First_num = 0;
    double minB = 0;
    int G[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    int G_size = 0;
    int G_Obstacles_[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (i = 0; i < 9; i++)
    {
        // temp 需要吗
        int temp = 0;
        DPoint r2o = Obstacles_.at(i) - robot_pos_;
        if (r2t.cross(r2o) == 0)
            temp = 0;
        else if (r2t.cross(r2o) > 0)
            temp = 1;
        else
            temp = -1;
        cos_cast[i] = r2t.ddot(r2o) / r2t.length();
        sin_cast[i] = temp * fabs(r2t.cross(r2o)) / r2t.length();
    }
    // obtain B that may hit
    //同一方向，在目标点与机器人中间，距离路线小于100
    for (i = 0; i < 9; i++)
    {
        if ((cos_cast[i] > 0) && (cos_cast[i] < r2t.length()) && (fabs(sin_cast[i]) < (radius_robot + radius_Obs)))
        {
            B[j] = i;
            j++;
        }
    }
    if (j != 0)
    {
        // determine first Obstacles_
        First_num = B[0];
        minB = cos_cast[B[0]];
        for (i = 1; i < j; i++)
        {
            if (minB < cos_cast[B[i]])
                minB = minB;
            else
            {
                minB = cos_cast[B[i]];
                First_num = B[i];
            }
        }
        // Grouping--the Obstacles_ that must be avoided
        G[0] = First_num;
        G_size = 0;
        G_Obstacles_[First_num] = 0;
        for (i = 0; i < 9; i++)
        {
            if (G_Obstacles_[i] == 1)
            {
                for (k = 0; k <= G_size; k++)
                {
                    if (Obstacles_.at(i).distance(Obstacles_.at(G[k])) < (2 * radius_robot + 2 * radius_Obs))
                    {
                        G_size++;
                        G[G_size] = i;
                        G_Obstacles_[i] = 0;
                        i = -1;
                        break;
                    }
                }
            }
        }

        relocate(G_size, cos_cast, sin_cast, G, Obstacles_, r2t);
    }
    else
        m_subtarget = target_pos_;
}

class movePID
{
public:
    movePID() : clock(0), Kp(8.0), Td(0.05), ret(0.0f) {}
    float PID_operation(const DPoint &target, const DPoint &cur);

private:
    int clock;
    double Kp;
    double Td;
    float ret;
};

float movePID::PID_operation(const DPoint &target, const DPoint &robot_pos_)
{
    if (clock >= 10)
        clock = 0;

    auto err = target.distance(robot_pos_);

    static auto pre_err = err;

    if (clock == 0)
    {
        //重新计算
        if (err >= 450.0)
        {
            ret = 500.0f;
        }
        else
        {
            ret = Kp * (err + Td * (pre_err - err) / 0.03);
        }
    }
    pre_err = err;
    ++clock;
    return ret;
}

movePID mpid;

class rotatePID
{
public:
    rotatePID() : clock(0), Kp(12.0), Td(0.10), ret(0.0f) {}
    float PID_operation(double target, double angle);

private:
    int clock;
    double Kp;
    double Td;
    float ret;
};

float rotatePID::PID_operation(double target, double angle)
{
    if (clock >= 10)
        clock = 0;
    auto err = fabs(target - angle);

    static auto pre_err = err;
    if (clock == 0)
    {
        //重新计算
        if (err >= 75.0 / 180 * SINGLEPI_CONSTANT)
        {
            ret = 6.0f;
        }
        else
        {
            ret = Kp * (err + Td * (pre_err - err) / 0.03);
        }
    }
    pre_err = err;
    ++clock;
    return ret;
}

rotatePID rpid;

// class _pid
// {
// public:
//     _pid() : _kp(0.0), _ki(0.6), _kd(0.4) {}
//     double ki() { return _ki; }
//     double kd() { return _kd; }
//     virtual double kp(double err) = 0;

// protected:
//     double _kp, _ki, _kd;
// };

// class Movepid : public _pid
// {
// public:
//     double kp(double err)
//     {
//         if (err >= 1500.0)
//             _kp = 8.0;
//         else if (err >= 1000)
//             _kp = 7.5 + (err - 1000) * 0.001;
//         else if (err > 500)
//             _kp = 5.0 + (err - 500) * 0.005;
//         else
//             _kp = 5.0;
//         return _kp;
//     }
// };

// class Rotatepid : public _pid
// {
// public:
//     double kp(double err)
//     {
//         err *= (180.0 / SINGLEPI_CONSTANT);
//         if (err > 120.0)
//             _kp = 6.0;
//         else if (err > 60.0)
//             _kp = 4.5 + (err - 60) * 0.025;
//         else if (err > 30.0)
//             _kp = 3.0 + (err - 30) * 0.05;
//         else
//             _kp = 3.0;
//         return _kp;
//     }
// };

// move slowly
bool Behaviour::move2target_slow(DPoint &target, DPoint &rob_pos, double err)
{
    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = rob_pos.distance(target);
    if (rob_pos.distance(target) > err)
        return false;
    return true;
}

bool Behaviour::move2targetk(DPoint target, DPoint pos, double distance_thres, int k) // 一个十分简单的实现，可以用PID
{
    subtarget(target, robot_pos_);
    target = m_subtarget;

    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = k * pos.distance(target);
    if (pos.distance(target) > distance_thres)
        return false;
    else
        return true;
}

bool Behaviour::move2oriFast(double target, double angle, double angle_thres)
{

    action->target_ori = target;
    action->maxw = rpid.PID_operation(target, angle);

    if (fabs(target - angle) > angle_thres)
        return false;

    return true;
}

//无避障快速移动
bool Behaviour::move2targetFast(DPoint target, DPoint pos, double distance_thres, int k)
{
    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = k * pos.distance(target);
    if (pos.distance(target) > distance_thres)
        return false;
    else
        return true;
}

bool Behaviour::move2target(DPoint target, DPoint pos, double distance_thres)
{
    subtarget(target, robot_pos_);
    target = m_subtarget;

    // shared_ptr<_pid> moveControl = make_shared<Movepid>();
    // // static _pid *moveControl = new Movepid;
    // static int cnt = 0;
    // static double sum(0);
    // if (cnt >= 50)
    // {
    //     cnt = 0;
    //     sum = 0;
    // }
    // static double err;
    // static double pre_err = pos.distance(target);
    // err = pos.distance(target);
    // //积分
    // sum += err;
    // ++cnt;
    // auto ierr = sum / cnt;

    // //微分
    // double derr = (err - pre_err);

    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = mpid.PID_operation(target, robot_pos_);
    // pre_err = err;
    if (pos.distance(target) > distance_thres)
        return false;
    else
        return true;
}

void Behaviour::selfRotate(double angle)
{
    static double target = SINGLEPI_CONSTANT;

    action->target_ori = target;
    action->maxw = 24.0;

    if (fabs(target - angle) <= 0.12)
        target = -target;
}

bool Behaviour::move2orif(double target, double angle, double angle_thres)
{
    action->target_ori = target;
    action->maxw = rpid.PID_operation(target, angle);

    if (fabs(target - angle) > angle_thres)
        return false;

    return true;
}

bool Behaviour::calPassingError(DPoint passRobot, DPoint catchRobot, double halfLength)
{
    DPoint c2p = catchRobot - passRobot;
    double err = atan2(halfLength, c2p.length());

    if (fabs(robot_ori_.radian_ - c2p.angle().radian_) <= err)
        return true;
    else
        return false;
}

//快速旋转
bool Behaviour::move2oriFAST(double t2r, double angle, double angle_thres, DPoint tar_pos, double tar_half_length, double speedup)
{ // angle_thres如果给0的话就用half_length判断模式 若用默认值则用角度差模式;half_length就是希望的误差长度范围的一半（自动计算误差角度）     0<= speedup<= 100
    double tempTar = t2r;
    double theta_e = tempTar - angle;
    while (theta_e > SINGLEPI_CONSTANT)
        theta_e = theta_e - 2 * SINGLEPI_CONSTANT;
    while (theta_e <= -SINGLEPI_CONSTANT)
        theta_e = theta_e + 2 * SINGLEPI_CONSTANT; //转化成-pi到pi的角度

    if (theta_e < 60.0 * DEG2RAD && theta_e > 0.0) //正角太小，加大
    {
        action->target_ori = t2r + speedup * DEG2RAD;
        action->maxw = fabs(t2r + speedup * DEG2RAD - angle) * 2;
    }
    else if (theta_e > -60.0 * DEG2RAD && theta_e < 0.0) //负角太小，加大
    {
        action->target_ori = t2r - speedup * DEG2RAD;
        action->maxw = fabs(t2r - speedup * DEG2RAD - angle) * 2;
    }
    else //正常转动
    {
        action->target_ori = t2r;
        action->maxw = fabs(t2r - angle) * 2;
    }
    if (fabs(angle_thres) < 1e-6 * DEG2RAD) // half_length模式
    {
        if (calPassingError(robot_pos_, tar_pos, tar_half_length)) //计算误差内，返回真值
            return true;
        else
            return false;
    }
    else
    {
        if (fabs(t2r - angle) > angle_thres) //角度差模式
            return false;
        else
            return true;
    }
}
