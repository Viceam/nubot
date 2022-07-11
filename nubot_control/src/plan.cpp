#include "nubot/nubot_control/plan.h"
#include "core.hpp"
using namespace nubot;

Plan::Plan() : our_goal(DPoint(-1100.0, 0.0)), opp_goal(DPoint(1100.0, 0.0))
{
	isinposition_ = false;
	action = nullptr;
	for (int i = 0; i < 4; ++i)
	{
		our_near_ball_sort_ID[i] = opp_near_ball_sort_ID[i] = i + 2;
	}
}

void Plan::update_a2b_radian()
{
	for (int i = 0; i < 5; i++)
	{
		past_radian[i].push(ball_pos_ - world_model_->Opponents_[i]);
		if (past_radian[i].size() > 6)
			past_radian[i].pop();
	}
}

void Plan::update_opp_location()
{
	for (int i = 0; i < 5; i++)
	{
		opp_location[i].push(world_model_->Opponents_[i]);
		if (opp_location[i].size() > 6)
			opp_location[i].pop();
	}
}

void Plan::update_past_velocity()
{
	past_velocity.push(ball_vel_);
	if (past_velocity.size() > 10)
		past_velocity.pop();
}

DPoint Plan::get_my_vel()
{
	my_location.push(robot_pos_);
	if (my_location.size() > 6)
		my_location.pop();
	DPoint my_vel(0, 0);
	if (my_location.size() != 6)
		return my_vel;
	double v_x, v_y;
	std::queue<DPoint> q = my_location;
	int t = 0;
	DPoint my_pos[6], s;
	while (!q.empty())
	{
		s = q.front();
		my_pos[t++] = s;
		q.pop();
	}
	for (int k = 0; k < 3; k++)
	{
		v_x += (my_pos[k + 3].x_ - my_pos[k].x_) / 3;
		v_y += (my_pos[k + 3].y_ - my_pos[k].y_) / 3;
	}
	my_vel.x_ = v_x / 0.09;
	my_vel.y_ = v_y / 0.09;
	return my_vel;
}

DPoint Plan::get_opp_vel(int i)
{
	DPoint opp_vel(0, 0);
	if (opp_location[i].size() != 6)
		return opp_vel;
	double v_x, v_y;
	std::queue<DPoint> q = opp_location[i];
	int t = 0;
	DPoint opp_pos[6], s;
	while (!q.empty())
	{
		s = q.front();
		opp_pos[t++] = s;
		q.pop();
	}
	for (int k = 0; k < 3; k++)
	{
		v_x += (opp_pos[k + 3].x_ - opp_pos[k].x_) / 3;
		v_y += (opp_pos[k + 3].y_ - opp_pos[k].y_) / 3;
	}
	opp_vel.x_ = v_x / 0.09;
	opp_vel.y_ = v_y / 0.09;
	return opp_vel;
}

double Plan::get_opp_ori(int i)
{
	if (past_radian[i].size() != 10)
		return 0;
	std::queue<DPoint> a2b = past_radian[i];
	int t = 0;
	double radian[10], delta_radian;
	while (!a2b.empty())
	{
		radian[t++] = a2b.front().angle().radian_;
		a2b.pop();
	}
	for (int k = 0; k < 5; k++)
	{
		delta_radian += (radian[k + 5] - radian[k]) / 5;
	}
	delta_radian /= 5;
	return delta_radian / 0.03;
}

void Plan::predict_attackerwithball(int i)
{
	DPoint attacker_pos_now = world_model_->Opponents_[i];
	DPoint attacker_vel = get_opp_vel(i);
	double a2b_radian = (ball_pos_ - attacker_pos_now).angle().radian_;
	double a2b_ori = get_opp_ori(i);
	double a2b_radian_next;
	attacker_next_pos = attacker_pos_now + 0.3 * attacker_vel;
	a2b_radian_next = a2b_radian + 0.3 * a2b_ori;
	a2b_next = DPoint(cos(a2b_radian_next), sin(a2b_radian_next));
}

// catchball , useless
void Plan::catchBallSlowly()
{
	auto r2b = ball_pos_ - robot_pos_;
	if (m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_))
	{
		m_behaviour_.move2target_slow(ball_pos_, robot_pos_);
		action->move_action = CatchBall_slow;
		action->rotate_acton = CatchBall_slow;
		action->rotate_mode = 0;
	}
}

void Plan::init_dis()
{
	for (int i = 0; i < 5; ++i)
		for (int j = 0; j < 5; ++j)
			dis[i][j] = world_model_->Opponents_[i].distance(world_model_->RobotInfo_[j].getLocation());
}

void Plan::sortourID()
{
	for (int i = 2; i >= 0; i--)
	{
		for (int j = 0; j <= i; j++)
		{
			double jlen = (world_model_->RobotInfo_[(our_near_ball_sort_ID[j]) - 1].getLocation() - ball_pos_).length();
			double jpluslen = (world_model_->RobotInfo_[(our_near_ball_sort_ID[j + 1]) - 1].getLocation() - ball_pos_).length();
			if (ifParking(world_model_->RobotInfo_[(our_near_ball_sort_ID[j]) - 1].getLocation()))
			{
				jlen += 4000.0;
			}
			if (ifParking(world_model_->RobotInfo_[(our_near_ball_sort_ID[j + 1]) - 1].getLocation()))
			{
				jpluslen += 4000.0;
			}
			if (jlen > jpluslen)
			{
				std::swap(our_near_ball_sort_ID[j + 1], our_near_ball_sort_ID[j]);
			}
		}
	}
}

void Plan::sortoppID()
{
	for (int i = 2; i >= 0; i--)
	{ // sort the length of opp robots' distances to ball, the [0] is the nearest
		for (int j = 0; j <= i; j++)
		{
			double jlen = (world_model_->Opponents_[(opp_near_ball_sort_ID[j]) - 1] - ball_pos_).length();
			double jpluslen = (world_model_->Opponents_[(opp_near_ball_sort_ID[j + 1]) - 1] - ball_pos_).length();
			if (ifParking(world_model_->Opponents_[(opp_near_ball_sort_ID[j]) - 1]))
			{
				jlen += 4000.0;
			}
			if (ifParking(world_model_->Opponents_[(opp_near_ball_sort_ID[j + 1]) - 1]))
			{
				jpluslen += 4000.0;
			}
			if (jlen > jpluslen)
			{
				std::swap(opp_near_ball_sort_ID[j + 1], opp_near_ball_sort_ID[j]);
			}
		}
	}
}

int Plan::Get_Ball_Mode(int state)
{
	static int loop_num_ = 6;
	static double start_vel_ = ball_vel_.length(); // record the vel last

	if (state == 1) // start mode
	{
		// init it
		start_vel_ = ball_vel_.length();
		loop_num_ = 6;
	}
	else // test mode
	{
		if (loop_num_ == 0)
		{
			int shoot_mode;
			if (fabs(ball_vel_.length() - start_vel_) < 1.0)
			{
				shoot_mode = FLY;
			}
			else
			{
				shoot_mode = RUN;
			}
			return shoot_mode;
		}
		else
		{
			loop_num_--;
			return 0; // is waiting and test
		}
	}
}

int Plan::ourDribble()
{
	for (int i = 0; i < 5; ++i)
		if (world_model_->RobotInfo_[i].getDribbleState())
			return i;
	return -1;
}

const DPoint Plan::start_point()
{
	static DPoint begin_point = DPoint(10000.0, 10000.0);
	if (ourDribble() > -1 || oppDribble() > -1)
	{
		begin_point = ball_pos_;
	}
	return begin_point;
}

bool Plan::ball_is_flying()
{
	if (past_velocity.size() != 10)
		return false;
	std::queue<DPoint> q = past_velocity;
	DPoint v;
	double past_velocity_[20], average_delta_v, maxvel = 0.0;
	int t = 0;
	while (!q.empty())
	{
		v = q.front();
		past_velocity_[++t] = sqrt(v.x_ * v.x_ + v.y_ * v.y_);
		q.pop();
	}
	for (int i = 1; i <= 10; i++)
	{
		if (past_velocity_[i] >= maxvel)
			maxvel = past_velocity_[i];
	}
	if (maxvel <= 0.1)
		return false;
	for (int i = 1; i <= 5; i++)
	{
		average_delta_v += (past_velocity_[i + 5] - past_velocity_[i]) / 5;
	}
	average_delta_v /= 5;
	return std::abs(average_delta_v) <= 0.01;
}

const DPoint Plan::landing_pos()
{
	DPoint flying_ball_pos;
	DPoint begin_point = start_point();
	flying_ball_pos = DPoint(-10000.0, -10000.0);
	if (ball_is_flying())
	{
		flying_ball_pos = begin_point + (ball_vel_.length() / 26.0) * (ball_vel_.length() / 26.0) / (ball_vel_.length()) * ball_vel_;
	}
	return flying_ball_pos;
}

const DPoint Plan::get_fly_landing_point()
{
	static double vel_change = ball_vel_.length();
	static int mode = 0; // 0 is waiting mode, 1 is record mode
	static DPoint begin(0.0, 0.0);
	static DPoint target(0.0, 0.0);

	if (mode == 0 && (ball_vel_.length() - vel_change) > 100.0) // begin to kick ball
	{
		begin = ball_pos_;
		mode = 1;
		Get_Ball_Mode(1);
		target = DPoint(-10000.0, -10000.0);
	}
	else if (mode == 1)
	{
		int ball_mode = Get_Ball_Mode(0);
		int Flg = 1; // 1 present can be determined as shoot, 0 present can not

		for (int i = 0; i < 5; i++)
		{
			if ((world_model_->RobotInfo_[i].getLocation() - ball_pos_).length() < 40.0)
			{
				Flg = 0;
			}
		}

		for (int i = 0; i < 5; i++)
		{
			if ((world_model_->Opponents_[i] - ball_pos_).length() < 40.0)
			{
				Flg = 0;
			}
		}

		if (ball_mode == FLY && Flg == 1)
		{
			target = begin + (ball_vel_.length() / 26.0) * (ball_vel_.length() / 26.0) / (ball_vel_.length()) * ball_vel_;
			mode = 0; // init state
			ROS_INFO("target = (%.1lf, %.1lf)", target.x_, target.y_);
		}
		else if (ball_mode == FLY && Flg == 0)
		{
			target = DPoint(-10000.0, -10000.0); // useless
			mode = 0;							 // init state
		}
		else if (ball_mode == RUN)
		{
			target = DPoint(-10000.0, -10000.0); // useless
			mode = 0;							 // init state
		}
		else
		{
			target = DPoint(-10000.0, -10000.0); // useless
		}

		ROS_INFO("ball_shoot_mode_ = %d", ball_mode);
	}
	else
	{
		target = DPoint(-10000.0, -10000.0); // useless
	}

	vel_change = ball_vel_.length();

	return target;
}

int Plan::oppDribble() // 0 - 4
{
	for (int i = 0; i < 5; ++i)
		if (world_model_->Opponents_[i].distance(ball_pos_) < 35.5)
			return i;
	return -1;
}

bool Plan::ball_is_free()
{
	for (int i = 0; i < 5; ++i)
	{
		if (world_model_->RobotInfo_[i].getDribbleState())
			return false;
	}
	return oppDribble() < 0;
}

int Plan::nearest_oppdribble()
{
	//改为距球位置

	// auto opp_pos_ = world_model_->Opponents_[oppDribble()];
	double min = MAX;
	int min_id(0);
	for (int i = 1; i < 5; ++i) // exclude goalkeeper
	{
		if (defend_occupied[i])
			continue;
		auto rob_pos_ = world_model_->RobotInfo_[i].getLocation();
		if (rob_pos_.distance(ball_pos_) < min)
		{
			min = rob_pos_.distance(ball_pos_);
			min_id = i;
		}
	}
	defend_occupied[min_id] = 1;
	return min_id;
}

int Plan::numofinOurPenalty()
{
	int num(0);
	for (int i = 1; i < 5; ++i)
		if (world_model_->field_info_.isOurPenalty(world_model_->RobotInfo_[i].getLocation()))
			++num;
	return num;
}

int Plan::opp_nearestToOurGoal() // 1 - 4
{
	for (int i = 1; i < 5; ++i)
	{
		//只能有一名进攻球员在我方禁区
		if (world_model_->field_info_.isOurPenalty(world_model_->Opponents_[i]))
			return i;
	}

	//无对方球员在禁区
	auto goal = DPoint(-1100.0, 0.0);
	double min = MAX;
	int min_id(0);
	for (int i = 1; i < 5; ++i)
	{
		if (world_model_->Opponents_[i].distance(goal) < min)
		{
			min = world_model_->Opponents_[i].distance(goal);
			min_id = i;
		}
	}

	return min_id;
}

int Plan::Snd_nearest_oppdribble()
{
	auto opp_pos_ = world_model_->Opponents_[oppDribble()];
	double min = MAX;
	int min_id(0);

	int first = nearest_oppdribble();

	for (int i = 1; i < 5; ++i) // exclude goalkeeper
	{
		if (i == first)
			continue;
		auto rob_pos_ = world_model_->RobotInfo_[i].getLocation();
		if (rob_pos_.distance(opp_pos_) < min)
		{
			min = rob_pos_.distance(opp_pos_);
			min_id = i;
		}
	}
	return min_id;
}

// catch ball function , only passing ball
void Plan::CatchPassedBall()
{
	if (oppDribble() >= 0)
	{
		world_model_->pass_state_.reset();
		return;
	}

	DPoint r2b = ball_pos_ - robot_pos_;

	action->rotate_acton = Positioned;
	action->move_action = No_Action;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);

	DPoint headoffPoint = robot_pos_;
	//垂直运动球路径

	if (ball_vel_.length())
	{
		action->move_action = CatchBall;
		double k1 = ball_vel_.y_ / ball_vel_.x_;
		double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
		double k2 = -1.0 / k1;
		double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
		headoffPoint = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2)); //交点
		if (m_behaviour_.move2target(headoffPoint, robot_pos_))
		{
			//接球
			if (robot_pos_.distance(ball_pos_) <= 50.0 && ball_is_free())
			{
				action->handle_enable = 1;
				action->move_action = CatchBall;
				action->rotate_acton = CatchBall;
				action->rotate_mode = 0;
				if (m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_, 0.087))
					m_behaviour_.move2target(ball_pos_, robot_pos_);
			}
		}
	}
}

// pass ball function
bool Plan::PassBall_Action(int catch_ID, int pass_mode_)
{

	// ROS_INFO("in pass fun %d", catch_ID);

	world_model_->pass_ID_ = world_model_->AgentID_;
	world_model_->catch_ID_ = catch_ID;
	world_model_->catch_pt_ = world_model_->RobotInfo_[catch_ID - 1].getLocation();
	bool shoot_flag = false;
	bool kick_off = false;

	// modify
	world_model_->is_passed_out_ = true;
	world_model_->pass_cmds_.isvalid = true;
	// m end
	DPoint pass_target_ = world_model_->RobotInfo_[catch_ID - 1].getLocation() - robot_pos_;
	if (m_behaviour_.move2oriFAST(pass_target_.angle().radian_, robot_ori_.radian_, 0.087))
	{
		world_model_->is_passed_out_ = true;
		world_model_->pass_cmds_.isvalid = true;
		world_model_->is_static_pass_ = 1; //先暂时设置静态传球
		action->shootPos = pass_mode_;

		if (pass_mode_ == -1) // FLY
			action->strength = sqrt((pass_target_.length() / 100.0) * 9.8);
		else
		{
			action->strength = action->strength = pass_target_.length() / 30;
			if (action->strength < 5.0)
				action->strength = 5.0;
		}

		shoot_flag = true;
		std::cout << "pass out" << std::endl;
		//        ROS_INFO("I will pass to %d,the pass mode is %d,his position is x::= %lf, y:==%lf,my position is x==%lf,  y==%lf",catch_ID,pass_mode_,world_model_->RobotInfo_[catch_ID-1].getLocation().x_,world_model_->RobotInfo_[catch_ID-1].getLocation().y_,world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().x_,world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().y_);
		kick_off = true;
	}
	return shoot_flag;
}

void Plan::ProtectBallTry()
{
	double OppAngle[5];
	int i = 0, j = 0;
	for (DPoint op : world_model_->Opponents_)
	{
		DPoint tmp = op - robot_pos_;
		if (tmp.length() <= 150.0)
		{
			OppAngle[i] = tmp.angle().radian_;
			i++;
		}
	}
	double avg = 0.0;
	for (j = 0; j < i; j++)
	{
		avg += OppAngle[j];
	}
	avg /= i;
	m_behaviour_.move2oriFAST(avg + 180 * DEG2RAD, robot_ori_.radian_);
	action->move_action = MoveWithBall;
	action->rotate_acton = MoveWithBall;
	action->rotate_mode = 0;
}

void Plan::catchBall()
{
	auto r2b = ball_pos_ - robot_pos_;
	action->handle_enable = 1;
	action->move_action = action->move_action = CatchBall;
	action->rotate_mode = 0;

	if (robot_pos_.distance(ball_pos_) >= 200)
	{
		m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_, 0.087);
		m_behaviour_.move2target(ball_pos_, robot_pos_);
	}

	else
	{
		if (m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_, 0.087))
			m_behaviour_.move2targetFast(ball_pos_, robot_pos_);
	}
}

bool Plan::moveBall(DPoint target)
{
	//中转
	DPoint subtarget = startPoint.pointofline(target, 270.0);
	action->move_action = MoveWithBall;
	action->rotate_acton = MoveWithBall;
	action->rotate_mode = 0;

	//真目标
	auto realTarget = (target.distance(startPoint) <= 270.0) ? target : subtarget;
	auto r2rt = realTarget - robot_pos_;

	if (m_behaviour_.move2oriFAST(r2rt.angle().radian_, robot_ori_.radian_))
	{
		m_behaviour_.move2target(realTarget, robot_pos_);
	}

	if (robot_pos_.distance(subtarget) <= 25.0)
	{
		action->move_action = action->move_action = TurnForShoot;
		action->shootPos = 1;
		action->strength = 0.5f;
	}

	return (robot_pos_.distance(target) <= 25.0);
}

void Plan::shoot_1(bool &shootFlg)
{
	const auto &goal_up = world_model_->field_info_.oppGoal_[GOAL_UPPER];
	const auto &goal_low = world_model_->field_info_.oppGoal_[GOAL_LOWER];
	static auto shoot_target = (world_model_->Opponents_[0].y_ > 0) ? goal_low : goal_up;

	action->move_action = TurnForShoot;
	action->rotate_acton = TurnForShoot;
	action->rotate_mode = 0;
	auto shoot_line = shoot_target - robot_pos_;

	m_behaviour_.move2oriFAST(shoot_line.angle().radian_, robot_ori_.radian_, 0.087, {0.0, 0.0}, 20.0, 40.0);

	//求出球的运动轨迹方程
	double t_x = robot_pos_.x_, t_y = robot_pos_.y_;
	auto k = tan(robot_ori_.radian_);
	double b = t_y - k * t_x;
	double y0 = 1100.0 * k + b;
	bool b1 = robot_ori_.radian_ / DEG2RAD >= -90.0 && robot_ori_.radian_ / DEG2RAD <= 90.0;
	if (b1 && y0 <= 95 && y0 >= -95 && fabs(y0 - world_model_->Opponents_[0].y_) >= 67.0)
	{
		action->shootPos = RUN;
		action->strength = 500;
		shootFlg = true;
		std::cout << "shoot done " << std::endl;
	}

	if (fabs(shoot_line.angle().radian_ - robot_ori_.radian_) <= 0.10)
	{
		shoot_target = (shoot_target == goal_up) ? goal_low : goal_up;
	}
}

int Plan::canPass(int catchID)
{
	DPoint p2c = world_model_->RobotInfo_[catchID - 1].getLocation() - robot_pos_;
	int ret = 1;
	//对方
	for (int i = 0; i < 5; ++i)
	{
		DPoint obs = world_model_->Opponents_[i];
		DPoint p2obs = obs - robot_pos_;

		//同向且在pass 与 catch之间
		if (p2c.ddot(p2obs) > 0 && (p2c.ddot(p2obs) / p2c.length()) < p2c.length())
		{
			//距球路线小于100;
			if (p2c.cross(p2obs) / p2c.length() < 100.0)
			{
				auto k = p2c.length() / (p2c.ddot(p2obs) / p2c.length());
				//可吊射
				if (k <= 2.5 && k >= 1.67)
					ret = -1;

				else
					return 0;
			}
		}
	}

	//己方 ???
	// for (int i = 0; i < 5; ++i)
	// {
	// 	if (i == world_model_->AgentID_ - 1 || i == catchID - 1)
	// 		continue;
	// 	DPoint p2obs = world_model_->RobotInfo_[i].getLocation() - robot_pos_;

	// 	if (p2c.ddot(p2obs) >= 0)
	// 	{
	// 		DPoint obs = world_model_->RobotInfo_[i].getLocation();
	// 		DPoint p2obs = obs - robot_pos_;

	// 		if (p2c.ddot(p2obs) >= 0 && (p2c.ddot(p2obs) / p2c.length()) < p2c.length())
	// 		{
	// 			//距球路线小于100;
	// 			if (p2c.cross(p2obs) / p2c.length() < 100.0)
	// 			{
	// 				auto k = p2c.length() / (p2c.ddot(p2obs) / p2c.length());
	// 				//可吊射
	// 				if (k <= 2.5 && k >= 1.67)
	// 					ret = -1;

	// 				else
	// 					return 0;
	// 			}
	// 		}
	// 	}
	// }

	if (ret == 1)
	{
		if (p2c.length() >= 700.0)
			return -1;
	}

	return ret;
}

void Plan::defend1v1()
{

	///加入预测

	DPoint opp_dribbling_pos_ = world_model_->Opponents_[oppDribble()];
	// DPoint opp2b = ball_pos_ - opp_dribbling_pos_;
	// Angle oppd_ori_ = opp2b.angle(); // 带球机器人朝向角

	auto target = opp_dribbling_pos_.pointofline(ball_pos_, 150.0);
	auto r2b = ball_pos_ - robot_pos_;
	auto r2opp = opp_dribbling_pos_ - robot_pos_;
	int att = oppDribble();
	DPoint target_;
	if (!ball_is_free())
	{
		// action->move_action = CatchBall;
		// action->rotate_acton = CatchBall;
		// action->rotate_mode = 0;
		// m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
		// m_behaviour_.move2target(target, robot_pos_);
		DPoint opp_vel_ = get_opp_vel(att);
		double opp_ori_ = get_opp_ori(att);
		predict_attackerwithball(att);
		target_ = ball_pos_;
		DPoint goal(-1100, 0);
		DPoint a2goal = goal - attacker_next_pos;
		double angle_cos = a2goal.ddot(a2b_next) / (a2goal.length() * a2b_next.length());
		if (angle_cos > 0.5)
		{
			target_ = attacker_next_pos + 55 * a2b_next;
		}
		else
		{
			if (a2goal.cross(a2b_next) > 0)
			{
				DPoint ver(-a2goal.y_, a2goal.x_);
				double l = ver.length();
				double l2 = a2goal.length();
				target_ = (27.5 * 0.75 / l2) * a2goal + (47.63 * 0.75 / l) * ver + attacker_next_pos;
			}
			else if (a2goal.cross(a2b_next) < 0)
			{
				DPoint ver(a2goal.y_, -a2goal.x_);
				double l = ver.length();
				double l2 = a2goal.length();
				target_ = (27.5 * 0.75 / l2) * a2goal + (47.63 * 0.75 / l) * ver + attacker_next_pos;
			}
		}

		action->handle_enable = 1;
		action->move_action = CatchBall;
		action->rotate_acton = CatchBall;
		action->rotate_mode = 0;
		if (m_behaviour_.move2target(target_, robot_pos_))
			m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
	}

	else if (r2b.ddot(r2opp) > 0) //球还在他们中间
	{
		// try to block

		// move to trial vertically
		DPoint headoffPoint = robot_pos_;
		double k1 = ball_vel_.y_ / ball_vel_.x_;
		double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
		double k2 = -1.0 / k1;
		double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
		headoffPoint = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));

		if (m_behaviour_.move2target(headoffPoint, robot_pos_))
		{
			//接球
			if (robot_pos_.distance(ball_pos_) <= 50.0 && ball_is_free())
			{
				action->handle_enable = 1;
				action->move_action = CatchBall;
				action->rotate_acton = CatchBall;
				action->rotate_mode = 0;
				if (m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_, 0.087))
					m_behaviour_.move2target(ball_pos_, robot_pos_);
			}
		}
	}
}

int Plan::nearest_point(DPoint point)
{
	// int occupied = nearest_oppdribble();

	double min = MAX;
	int min_id;
	for (int i = 1; i < 5; ++i)
	{
		if (defend_occupied[i])
			continue;
		auto rob_pos = world_model_->RobotInfo_[i].getLocation();
		if (rob_pos.distance(point) < min)
		{
			min = rob_pos.distance(point);
			min_id = i;
		}
	}
	defend_occupied[min_id] = 1;
	return min_id;
}

int Plan::nearest_opp(int exp1, int exp2)
{
	int min = MAX;
	int min_id(-1);
	for (int i = 1; i < 5; ++i)
	{
		if(i == exp1 || i == exp2)
			continue;
		if (robot_pos_.distance(world_model_->Opponents_[i]) < min)
		{
			min = robot_pos_.distance(world_model_->Opponents_[i]);
			min_id = i;
		}
	}

	return min_id;
}

void Plan::block()
{

	DPoint r2b = ball_pos_ - robot_pos_;
	//距球门最近的对方机器人的位置
	DPoint opp_pos1 = world_model_->Opponents_[opp_nearestToOurGoal()];
	//对方带球机器人位置
	DPoint opp_pos2 = world_model_->Opponents_[oppDribble()];

	DPoint target;
	if (opp_pos1.distance(ball_pos_) <= 100.0)
		target = ball_pos_.pointofline(opp_pos1, 100.0);
	else
		target = opp_pos1.pointofline(opp_pos2, 100.0);

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);

	return;
}

void Plan::mark()
{
	int opp_dri_ = oppDribble();
	DPoint opp_pos2 = world_model_->Opponents_[opp_dri_];
	int opp_near_goal_ = opp_nearestToOurGoal();
	double ori = (ball_pos_ - world_model_->Opponents_[opp_dri_]).angle().radian_;
	DPoint r2b = ball_pos_ - robot_pos_;
	DPoint target;
	bool Flg = 0;
	int pass_id_ = -1;

	//找一个可能的接传球者
	
	for (int i = 1; i < 5; ++i)
	{
		if (i == opp_dri_ || i == opp_near_goal_)
			continue;
		double ori2 = (world_model_->Opponents_[i] - world_model_->Opponents_[opp_dri_]).angle().radian_;
		if (fabs(ori - ori2) <= 15.0 / DEG2RAD)
		{
			Flg = 1;
			pass_id_ = i;
		}
	}

	if (Flg)
	{
		//防守可能接球者
		auto opp_pos_ = world_model_->Opponents_[pass_id_];
		target = opp_pos_.pointofline(opp_pos2, 125.0);
	}

	else //防守离自己最近的机器人
	{
		//距自己最近的对方机器人的位置
		DPoint opp_pos1 = world_model_->Opponents_[nearest_opp(opp_dri_, opp_near_goal_)];

		// if(opp_pos1.distance(opp_pos2) <= 200.0) return;

		DPoint target = opp_pos1.pointofline(opp_pos2, 125.0);
	}

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);

	return;
}

int Plan::opp_getsforward()
{
	int min = MAX;
	int min_id(-1);
	for (int i = 1; i < 5; ++i)
	{
		if (world_model_->Opponents_[i].x_ < min)
		{
			min = world_model_->Opponents_[i].x_;
			min_id = i;
		}
	}
	return min_id;
}

double Plan::k2(const DPoint &p1, const DPoint &p2)
{
	return ((p1.y_ - p2.y_) / (p1.x_ / p2.x_));
}

void Plan::defend_shoot()
{
	DPoint target;
	DPoint opp_pos_ = opp_sort[0]; // 对方射门机器人位置
	if (opp_pos_.x_ <= -875)
	{
		target = ball_pos_.pointofline(opp_pos_, 75.0);
	}
	else
	{
		target.x_ = -1000.0;
		double k = k2(ball_pos_, opp_pos_);
		target.y_ = k * (-1000.0) + ball_pos_.y_ - k * ball_pos_.x_;
	}

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST((ball_pos_ - robot_pos_).angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2targetFast(target, robot_pos_);
}

void Plan::blockPassingBall()
{
	//寻找前插最深入的对方机器人
	DPoint opp_pos_ = world_model_->Opponents_[opp_getsforward()];
	// DPoint opp_pos_ = world_model_->Opponents_[opp_getsforward()]; //位置

	DPoint target = opp_pos_.pointofline(ball_pos_, 150.0);

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST((ball_pos_ - robot_pos_).angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);
}

void Plan::update_opp_sort()
{
	for (int i = 0; i < 4; ++i)
		opp_sort[i] = world_model_->Opponents_[opp_near_ball_sort_ID[i] - 1];
}

void Plan::update_near()
{
	double minlength = MAX;
	for (int i = 2; i <= 5; i++)
	{
		if (!ifParking(world_model_->RobotInfo_[i - 1].getLocation()) && i != our_near_ball_sort_ID[0]) // except our robot which is the nearest to ball
		{
			if ((world_model_->RobotInfo_[i - 1].getLocation() - opp_goal).length() < minlength)
			{
				our_nearest_oppgoal_ID = i;
				minlength = (world_model_->RobotInfo_[i - 1].getLocation() - opp_goal).length();
			}
		}
	}

	minlength = MAX;

	for (int i = 2; i <= 5; i++)
	{
		if (!ifParking(world_model_->RobotInfo_[i - 1].getLocation()) && i != our_near_ball_sort_ID[0]) // except our robot which is the nearest to ball
		{
			if ((world_model_->RobotInfo_[i - 1].getLocation() - our_goal).length() < minlength)
			{
				our_nearest_ourgoal_ID = i;
				minlength = (world_model_->RobotInfo_[i - 1].getLocation() - our_goal).length();
			}
		}
	}
}

void Plan::update_a()
{
	update_past_velocity();
	update_opp_location();
	update_a2b_radian();
	sortourID();
	sortoppID();
	update_opp_sort();
	update_near();
	landing_pos_ = landing_pos();
}

void Plan::pre_attack()
{
	DPoint pre_attack_pos_(500.0, 0.0);
	auto our_nearest_ball_pos = world_model_->RobotInfo_[our_near_ball_sort_ID[0] - 1].getLocation();
	if (ball_pos_.x_ - our_nearest_ball_pos.x_ != 0.0)
	{
		pre_attack_pos_.y_ = ball_pos_.y_ + (500.0 - ball_pos_.x_) * (ball_pos_.y_ - our_nearest_ball_pos.y_) / (ball_pos_.x_ - our_nearest_ball_pos.x_);
	}
	pre_attack_pos_.y_ = std::max(-600.0, pre_attack_pos_.y_);
	pre_attack_pos_.y_ = std::min(600.0, pre_attack_pos_.y_);
	int can_goto_point = 1;
	DPoint need_defend_pos;
	for (int i = 0; i < 5; i++)
	{
		if ((world_model_->Opponents_[i] - pre_attack_pos_).length() < 200.0)
		{
			can_goto_point = 0;
			need_defend_pos = world_model_->Opponents_[i];
			break;
		}
	}
	DPoint target;
	if (can_goto_point == 1) // can go to that point
		target = pre_attack_pos_;
	else // can not go to that point
	{
		target = need_defend_pos.pointofline(ball_pos_, 100.0);
	}

	auto r2b = ball_pos_ - robot_pos_;

	action->handle_enable = 1;
	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	if (m_behaviour_.move2target(target, robot_pos_))
		m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
}

void Plan::defend_point(DPoint target_pos_)
{
	DPoint target;
	auto r2b = ball_pos_ - robot_pos_;
	if (target_pos_.distance(ball_pos_) <= 150.0)
		target = ball_pos_.pointofline(target_pos_, 100.0);
	else
		target = target_pos_.pointofline(ball_pos_, 75.0);

	action->handle_enable = 1;
	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	if (m_behaviour_.move2target(target, robot_pos_))
		m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
}

void Plan::move2landingPos()
{
	auto r2b = ball_pos_ - robot_pos_;
	action->handle_enable = 1;
	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	if (m_behaviour_.move2targetFast(landing_pos_, robot_pos_)) //无避障
		m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
}

void Plan::move2catch(DPoint target)
{
	auto r2b = ball_pos_ - robot_pos_;
	action->handle_enable = 1;
	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	if (m_behaviour_.move2targetFast(target, robot_pos_)) //无避障
		m_behaviour_.move2oriFAST(r2b.angle().radian_, robot_ori_.radian_);
}


void Plan::defend()
{
	update_a();

	//处理传球
	if (world_model_->pass_state_.catch_id_ != -1)
	{
		if (world_model_->pass_state_.catch_id_ == world_model_->AgentID_)
		{
			CatchPassedBall();
		}
		else
		{
			attack();
		}
	}

	if (oppDribble() != -1) //对方带球
	{
		int opp_dri = oppDribble(), opp_near_goal = opp_nearestToOurGoal();
		auto opp_dri_pos = (opp_dri >= 0) ? (world_model_->Opponents_[opp_dri]) : DPoint(0.0, 0.0);
		auto opp_near_goal_pos = world_model_->Opponents_[opp_near_goal];

		if (ball_pos_.x_ <= 0) //球在我方半场
		{
			//带球人离球门最近
			if (opp_dri == opp_near_goal)
			{
				if (world_model_->AgentID_ - 1 == nearest_oppdribble())
				{
					defend_shoot();
				}

				else
				{
					mark();
				}
			}

			//带球人不离球门最近
			else
			{
				DPoint target = opp_near_goal_pos.pointofline(opp_dri_pos, 150.0);
				//是离带球对手最近的机器人
				if (world_model_->AgentID_ - 1 == nearest_oppdribble())
				{
					// 1-1防守
					defend1v1();
				}

				else if (world_model_->AgentID_ - 1 == nearest_point(target))
				{
					//封堵向球门附近的传球
					block();
				}

				else
				{
					//存储剩余的两个机器人
					int avail[2];
					for (int i = 1, j = 0; i < 5; ++i)
					{
						if (defend_occupied[i] == 0)
							avail[j++] = i;
					}

					if (world_model_->RobotInfo_[avail[0]].getLocation().x_ > world_model_->RobotInfo_[avail[1]].getLocation().x_)
					{
						std::swap(avail[0], avail[1]);
					}

					if (world_model_->AgentID_ - 1 == avail[0])
					{
						//防守可能的传球或离自己最近的敌方机器人
						mark();
					}

					else
					{
						//偷猎者 运动到便于接球与进攻的点
						pre_attack();
					}
					// mark();
				}
			}
		}

		else //球在对方半场
		{
			if (world_model_->AgentID_ - 1 == nearest_oppdribble())
			{
				defend1v1();
			}

			else if (world_model_->AgentID_ != our_nearest_ourgoal_ID)
			{
				//存储两可用机器人
				int avail[2];
				for (int i = 1, j = 0; i < 5; ++i)
				{
					if (i != our_nearest_ourgoal_ID - 1 && defend_occupied[i] == 0)
						avail[j++] = i;
				}
				//对方距球第二近的机器人位置
				auto opp_pos_ = world_model_->Opponents_[opp_near_ball_sort_ID[1]];

				if (world_model_->RobotInfo_[avail[0]].getLocation().distance(opp_pos_) > world_model_->RobotInfo_[avail[1]].getLocation().distance(opp_pos_))
				{
					std::swap(avail[0], avail[1]);
				}

				if (world_model_->AgentID_ - 1 == avail[0])
				{
					defend_point(opp_pos_);
				}
				else
				{
					defend_point(world_model_->Opponents_[opp_near_ball_sort_ID[2]]);
				}
			}

			//离我方球门最近
			else if (world_model_->AgentID_ == our_nearest_ourgoal_ID)
			{
				DPoint target;
				int Flg = 0;
				int defend_ID = -1;
				double minlength2 = MAX;
				for (int i = 2; i <= 5; ++i)
				{
					if (world_model_->Opponents_[i - 1].x_ < 0.0 && (world_model_->Opponents_[i - 1] - our_goal).length() < minlength2)
					{
						if (world_model_->Opponents_[i - 1].y_ > -700.0 && world_model_->Opponents_[i - 1].y_ < 700.0)
						{
							Flg = 1;
							defend_ID = i;
							minlength2 = (world_model_->Opponents_[i - 1] - our_goal).length();
						}
					}
				}

				if (Flg == 0) // there is no opprobot in our semi-ground
				{
					target.x_ = 0.0;
					target.y_ = ball_pos_.y_;
				}
				else // there is one or more than one opprobot in our semi-ground, then prevent the nearest one to our goal
				{
					target = world_model_->Opponents_[defend_ID - 1] + 100.0 / ((ball_pos_ - world_model_->Opponents_[defend_ID - 1]).length()) * (ball_pos_ - world_model_->Opponents_[defend_ID - 1]);
				}
				move2catch(target);
			}
		}
	}

	else // ball's free
	{
		if (ball_pos_.x_ <= 0) //球在我方半场
		{
			if (world_model_->field_info_.isOurPenalty(robot_pos_))
			{
				catchBall();
			}
			else
			{
				DPoint target;
				//当前机器人离球最近
				if (world_model_->AgentID_ == our_near_ball_sort_ID[0])
				{
					if (landing_pos_.x_ > -1200) // ball is flying
					{
						move2landingPos();
					}
					else
					{
						catchBall();
					}
				}

				else if (world_model_->AgentID_ != our_nearest_oppgoal_ID)
				{
					//是另外两个
					//存储
					int avail[2];
					for (int i = 1, j = 0; i < 5; ++i)
					{
						if (i != our_near_ball_sort_ID[0] - 1 && i != our_nearest_oppgoal_ID - 1)
							avail[j++] = i;
						// useless
						if (j == 2)
							break;
					}

					auto opp_pos_ = opp_sort[1]; //距球第二近的对方位置
					if (world_model_->RobotInfo_[avail[0]].getLocation().distance(opp_pos_) > world_model_->RobotInfo_[avail[1]].getLocation().distance(opp_pos_))
					{
						std::swap(avail[0], avail[1]);
					}
					if (world_model_->AgentID_ - 1 == avail[0])
					{
						defend_point(opp_sort[1]);
					}
					else
					{
						defend_point(opp_sort[2]);
					}
				}
				else if (world_model_->AgentID_ == our_nearest_oppgoal_ID)
				{
					pre_attack();
				}
			}
		}
		else //球在对方半场
		{
			if (world_model_->field_info_.isOppPenalty(robot_pos_))
			{
				catchBall();
			}
			else
			{
				if (world_model_->AgentID_ == our_near_ball_sort_ID[0])
				{
					catchBall();
				}

				else if (world_model_->AgentID_ != our_nearest_ourgoal_ID)
				{
					int avail[2];
					for (int i = 1, j = 0; i < 5; i++)
					{
						if (i != our_near_ball_sort_ID[0] - 1 && i != our_nearest_ourgoal_ID - 1)
						{
							avail[j++] = i;
						}
					}
					auto opp_pos_ = opp_sort[1];
					if (world_model_->RobotInfo_[avail[0]].getLocation().distance(opp_pos_) > world_model_->RobotInfo_[avail[1]].getLocation().distance(opp_pos_))
					{
						std::swap(avail[0], avail[1]);
					}

					if (world_model_->AgentID_ == avail[0])
					{
						defend_point(opp_pos_);
					}
					else
					{
						defend_point(opp_sort[2]);
					}
				}

				else //离我方球门最近
				{
					DPoint target;
					int Flg = 0;
					int defend_ID = 0;
					double minlength2 = 10000.0;
					for (int i = 2; i <= 5; i++)
					{
						if (world_model_->Opponents_[i - 1].x_ < 0.0 && (world_model_->Opponents_[i - 1] - our_goal).length() < minlength2)
						{
							if (world_model_->Opponents_[i - 1].y_ > -700.0 && world_model_->Opponents_[i - 1].y_ < 700.0)
							{
								Flg = 1;
								defend_ID = i;
								minlength2 = (world_model_->Opponents_[i - 1] - our_goal).length();
							}
						}
					}

					if (Flg == 0)
					{
						target = DPoint(0.0, ball_pos_.y_);
					}
					else // there is one or more than one opprobot in our semi-ground, then prevent the nearest one to our goal
					{
						target = world_model_->Opponents_[defend_ID - 1] + 100.0 / ((ball_pos_ - world_model_->Opponents_[defend_ID - 1]).length()) * (ball_pos_ - world_model_->Opponents_[defend_ID - 1]);
					}
					move2catch(target);
				}
			}
		}
	}

	return;
}

//............................................................................//

void Plan::attack()
{
	// auto target = DPoint(700, 0);
	return;
}
