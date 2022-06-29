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
    // if (clock >= 10)
    //     clock = 0;
    // auto err = fabs(target - angle);
    
    // static auto pre_err = err;
    // if (clock == 0)
    // {
    //     //重新计算
    //     if (err >= 75.0 / 180 * SINGLEPI_CONSTANT)
    //     {
    //         ret = 6.0f;
    //     }
    //     else
    //     {
    //         ret = Kp * (err + Td * (pre_err - err) / 0.03);
    //     }
    // }
    // pre_err = err;
    // ++clock;
    return 12.0f;
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

bool Behaviour::move2oriFast(double target, double angle, double angle_thres)
{
    
    action->target_ori = target;
    action->maxw = rpid.PID_operation(target, angle);

    if (fabs(target - angle) > angle_thres)
        return false;

    return true;
}

//守门员
bool Behaviour::move2targetFast(DPoint target, DPoint pos, double distance_thres)
{
    // subtarget(target, robot_pos_);
    // target = m_subtarget;
    action->target.x = target.x_;
    action->target.y = target.y_;
    action->maxvel = mpid.PID_operation(target, robot_pos_);
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
    
    if(fabs(target - angle) <= 0.12)
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
