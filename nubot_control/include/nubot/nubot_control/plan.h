#ifndef PLAN_H
#define PLAN_H

#include <cmath>
#include "core.hpp"
#include "nubot/nubot_control/subtargets.h"
#include "nubot/nubot_control/behaviour.hpp"
#include "nubot/nubot_control/world_model_info.h"

using namespace std;
namespace nubot
{
	class Plan
	{
	public:
		Plan();

		/***********catch ball***********/
		void catchBall();
		void catchBallForCoop();
		void catchBallSlowly();
		void catchMovingBall();
		void catchMotionlessBall();

		/***********postion*************/
		void positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta);
		void driblleControl(DPoint target, double acc, double sacc, double lvel, double maxvel);
		// void move2Positionwithobs(DPoint target);  // move to the target point with obstacles avoidance
		void move2Positionwithobs_noball(DPoint target, float maxvel, float maxacc, bool avoid_ball = false);

		/***********PE and PO***********/
		double PECrossBackMIdlleLine(double direction);
		double PEOutField(double direction);
		double PEInOurPenaty(double direction);
		double PObleDirection4OurField(double direction, double predictlen, double cobledirection, double kobledirection);
		double PObleDirection(double direction, double predictlen, double cobledirection, double kobledirection);

		/***********check***********/
		bool IsNullInTrap(double direction, double swidth, double lwidth, double len);
		/*********find and search*********/
		double FindBstDirectionForAvoid();
		double FindBstDirectionForAvoid2(DPoint target);
		int GetAvoidState();
		double SearchDirectionforMinPEPoint(double oridirection, double step, int lefttime, int righttime);
		bool SearchMinPE4PassThroughforOurField(double &direction, double pridictlen, DPoint trap[4], double step, int flg);
		bool SearchMinPE4PassThrough(double &direction, double pridictlen, DPoint trap[4], double step, int flg);
		void update();

	public:
		World_Model_Info *world_model_;
		Behaviour m_behaviour_;
		Subtargets m_subtargets_;

		float kp;
		float kalpha;
		float kbeta;
		bool inourfield_;
		bool inoppfield_;
		double lastdirection;

		DPoint robot_pos_;
		Angle robot_ori_;
		DPoint robot_vec_;
		DPoint ball_pos_;
		DPoint ball_vel_;
		vector<DPoint> target_;

	public:
		bool isinposition_;
		// add
		nubot_common::ActionCmd *action;
		DPoint startPoint;
		DPoint landing_pos_;
		const DPoint start_point();
		const DPoint landing_pos();
		bool PassBall_Action(int catch_ID, int pass_mode_);
		void CatchPassedBall(void);
		void ProtectBallTry();
		bool ifParking(DPoint pos)
		{
			return (pos.y_ >= 800.0 || pos.y_ <= -800.0);
		}
		bool ball_is_flying();
		bool opp_defended[5];
		bool defend_occupied[5];
		bool attack_occupied[5];
		void attack(); //进攻
		void defend(); //防守

		// dis[i][j] : 对方i号球员到我方j号球员的距离
		array<array<double, 5>, 5> dis;

		void init_dis();

		//以下待移入private

		void move2catch(DPoint);

		int ourDribble();
		//盯防
		void mark();
		//封堵 移动到对方两机器人之间
		void block();
		//阻挡射门
		void defend_shoot();
		//阻挡传球
		void blockPassingBall();
		//接落地球
		void move2landingPos();
		// 1v1 随带球机器人朝向进行移动
		void defend1v1();
		//运球
		bool moveBall(DPoint);
		//准备进攻
		void pre_attack();
		//防守某个位置的机器人
		void defend_point(DPoint);

		bool isMoving = false;
		// shoot
		void shoot_1(bool &);
		int canPass(int);
		const DPoint get_fly_landing_point();

	private:
		const double DEG2RAD = 0.01745329251994329547;
		const int RUN = 1;
		const int FLY = -1;
		const double MAX = 2147483647.0;
		const double MIN = -2147483647.0;

		queue<DPoint> past_velocity; // ball
		queue<DPoint> opp_location[5];
		queue<DPoint> my_location;
		queue<DPoint> past_radian[5]; // attacker to ball
		queue<int> last_dribble_state;

		//关键信息//
		int our_nearest_oppgoal_ID;
		int our_nearest_ourgoal_ID;
		DPoint opp_sort[4];
		int our_near_ball_sort_ID[4];
		int opp_near_ball_sort_ID[4];
		//关键信息//

	private:
		void update_a2b_radian();
		void update_opp_location();
		void update_past_velocity();
		void update_near();
		void sortourID();
		void update_opp_sort();
		void sortoppID();
		double k2(const DPoint& p1, const DPoint& p2);
		DPoint get_my_vel();
		DPoint get_opp_vel(int);
		double get_opp_ori(int);
		void predict_attackerwithball(int i);
		DPoint attacker_next_pos;
		DPoint a2b_next;
		const DPoint opp_goal;
		const DPoint our_goal;
		//离某个点最近
		int nearest_point(DPoint);

		//更新一些关键信息
		void update_a();

		// defend
		//返回离对方带球机器人最近的球员
		int nearest_oppdribble();
		//球是否自由
		bool ball_is_free();
		//对方几号带球
		int oppDribble();
		//距我方球门最近的敌方机器人
		int opp_nearestToOurGoal();
		//有几个己方机器人在己方禁区
		int numofinOurPenalty();
		//返回距离对方带球机器人第二近的球员
		int Snd_nearest_oppdribble();
		//返回前插最深入的对方机器人
		int opp_getsforward();
		//离自己最近的opp
		int nearest_opp(int exp1 = -1, int exp2 = -1);
		// defend end

		int Get_Ball_Mode(int);
	};
}
#endif // PLAN_H

