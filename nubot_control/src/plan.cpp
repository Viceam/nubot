#include "nubot/nubot_control/plan.h"
#include "core.hpp"
using namespace nubot;

Plan::Plan()
{
	isinposition_ = false;
	action = nullptr;
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

int Plan::oppDribble() // 0 - 4
{
	for (int i = 0; i < 5; ++i)
		if (world_model_->Opponents_[i].distance(ball_pos_) < 36)
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
	if (oppDribble() >= 0)
		return false;
	return true;
}

int Plan::nearest_oppdribble()
{
	auto opp_pos_ = world_model_->Opponents_[oppDribble()];
	double min = MAX;
	int min_id(0);
	for (int i = 1; i < 5; ++i) // exclude goalkeeper
	{
		if(defend_occupied[i])
			continue;
		auto rob_pos_ = world_model_->RobotInfo_[i].getLocation();
		if (rob_pos_.distance(opp_pos_) < min)
		{
			min = rob_pos_.distance(opp_pos_);
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
		action->move_action = Positioned;
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

		// ROS_INFO("strength: %lf ball_v %lf", action->strength, ball_vel_.length());
		// ROS_INFO("dajosiiiiiiiii\n"
		// 		 "sjiodfi*******\n"
		// 		 "aodjsii***iii\n");

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
			m_behaviour_.move2target(ball_pos_, robot_pos_);
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
	auto realTarget = (target.distance(startPoint) <= 270) ? target : subtarget;
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
	DPoint opp_dribbling_pos_ = world_model_->Opponents_[oppDribble()];
	// DPoint opp2b = ball_pos_ - opp_dribbling_pos_;
	// Angle oppd_ori_ = opp2b.angle(); // 带球机器人朝向角

	auto target = opp_dribbling_pos_.pointofline(ball_pos_, 100.0);

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST((ball_pos_ - robot_pos_).angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);
}

int Plan::nearest_point(DPoint point)
{
	//int occupied = nearest_oppdribble();

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

void Plan::block()
{

	DPoint r2b = ball_pos_ - robot_pos_;
	//距球门最近的对方机器人的位置
	DPoint opp_pos1 = world_model_->Opponents_[opp_nearestToOurGoal()];
	//对方带球机器人位置
	DPoint opp_pos2 = world_model_->Opponents_[oppDribble()];

	// if(opp_pos1.distance(opp_pos2) <= 200.0) return;

	DPoint target = opp_pos1.pointofline(opp_pos2, 125.0);

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
	for(int i = 1; i < 5; ++i)
	{
		if(world_model_->Opponents_[i].x_ < min)
		{
			min = world_model_->Opponents_[i].x_;
			min_id = i;
		}
	}
	return min_id;
}

void Plan::defend_shoot()
{
	DPoint opp_pos_ = world_model_->Opponents_[oppDribble()]; // 对方射门机器人位置
	DPoint target = opp_pos_.pointofline(ball_pos_, 75.0);
	
	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST((ball_pos_ - robot_pos_).angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);
}

void Plan::blockPassingBall()
{
	//寻找前插最深入的对方机器人
	DPoint opp_pos_ = world_model_->Opponents_[opp_getsforward()];//位置 

	DPoint target = opp_pos_.pointofline(ball_pos_, 150.0);

	action->move_action = CatchBall;
	action->rotate_acton = CatchBall;
	action->rotate_mode = 0;
	m_behaviour_.move2oriFAST((ball_pos_ - robot_pos_).angle().radian_, robot_ori_.radian_);
	m_behaviour_.move2target(target, robot_pos_);
}

void Plan::defend()
{
	int opp_dri = oppDribble(), opp_near_goal = opp_nearestToOurGoal();
	auto opp_dri_pos = world_model_->Opponents_[opp_dri], opp_near_goal_pos = world_model_->Opponents_[opp_near_goal];

	//带球人离球门最近或中间无空隙
	if (opp_dri_pos.distance(opp_near_goal_pos) <= 200.0)
	{
		if(world_model_->AgentID_ - 1 == )
		{
			
			defend_shoot();
		}
	}

	//带球人不离球门最近
	else
	{
		DPoint target = opp_near_goal_pos.pointofline(opp_dri_pos, 150.0);
		//是离带球对手最近的机器人
		if (world_model_->AgentID_ - 1 == nearest_oppdribble())
		{
			defend1v1();
		}

		else if (world_model_->AgentID_ - 1 == nearest_point(target))
		{
			block();
		}
	}
}

void Plan::attack()
{
	// auto target = DPoint(700, 0);
}