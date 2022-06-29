#include "nubot/nubot_control/plan.h"
#include "core.hpp"
using namespace nubot;

Plan::Plan()
{
	isinposition_ = false;
	action = nullptr;
}

// catchball useless
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

// void Plan::catchBall()
// {

// }

int Plan::oppDribble()
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
	m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_, 0.087);

	DPoint headoffPoint = robot_pos_;
	//垂直运动球路径

	if (ball_vel_.length())
	{
		action->move_action = Positioned;
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
				if (m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_))
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
	if (m_behaviour_.move2oriFast(pass_target_.angle().radian_, robot_ori_.radian_, 0.087))
	{
		world_model_->is_passed_out_ = true;
		world_model_->pass_cmds_.isvalid = true;
		world_model_->is_static_pass_ = 1; //先暂时设置静态传球
		action->shootPos = pass_mode_;

		if (pass_mode_ == -1) // FLY
			action->strength = sqrt((pass_target_.length() / 100.0 - 2.0) * 9.8);
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

void Plan::catchBall()
{
	auto r2b = ball_pos_ - robot_pos_;
	action->handle_enable = 1;
	action->move_action = action->move_action = CatchBall;
	action->rotate_mode = 0;
	if (m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_))
		m_behaviour_.move2target(ball_pos_, robot_pos_);
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

	if (m_behaviour_.move2orif(r2rt.angle().radian_, robot_ori_.radian_))
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

	m_behaviour_.move2orif(shoot_line.angle().radian_, robot_ori_.radian_, 0.09);

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
	
}

void Plan::attack_1()
{
	auto target = DPoint(700, 0);
}