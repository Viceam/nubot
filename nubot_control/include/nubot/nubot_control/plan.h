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
                bool PassBall_Action(int catch_ID, int pass_mode_);
                void CatchPassedBall(void);
                void ProtectBallTry();

                bool defend_occupied[5];
                bool attack_occupied[5];
                void attack(); //进攻
                void defend(); //防守

                //以下待移入private

                //void mark(); //盯防
                //封堵 移动到对方两机器人之间
                void block();
                //阻挡射门
                void defend_shoot(); 
                //阻挡传球
                void blockPassingBall();
                //1v1 随带球机器人朝向进行移动
                void defend1v1(); 
                //运球
                bool moveBall(DPoint);
                bool isMoving = false;
                //shoot
                void shoot_1(bool&);
                int canPass(int);
        private:
                const double DEG2RAD = 0.01745329251994329547;
                const int RUN = 1;
                const int FLY = -1;
                const double MAX = 2147483647.0; 
                const double MIN = -2147483647.0;
        private:

                //离某个点最近
                int nearest_point(DPoint);

                //defend
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
                
                //defend end
        };
}
#endif // PLAN_H
