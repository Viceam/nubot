#include "core.hpp"
#include <nubot_common/VelCmd.h>
#include <nubot_common/WorldModelInfo.h>
#include <nubot_common/BallInfo3d.h>
//#include <nubot_common/BallHandle.h>
//#include <nubot_common/Shoot.h>
#include <nubot_common/ActionCmd.h>
#include <nubot_common/BallIsHolding.h>
#include <nubot_common/StrategyInfo.h>
#include <nubot_common/TargetInfo.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <nubot/nubot_control/world_model_info.h>
#include <nubot/nubot_control/strategy.hpp>
#include <nubot/nubot_control/common.hpp>
#include <nubot/nubot_control/plan.h>
#include <nubot/nubot_control/staticpass.h>
#include <fstream>

#define RUN 1
#define FLY -1
const double DEG2RAD = 1.0 / 180.0 * SINGLEPI_CONSTANT; // 角度到弧度的转换

using namespace std;

ofstream file("data.sys", ios::app);

namespace nubot
{

    class NuBotControl
    {
    public:
        ros::Subscriber ballinfo3d_sub1_;
        ros::Subscriber odoinfo_sub_;
        ros::Subscriber obstaclesinfo_sub_;
        ros::Subscriber worldmodelinfo_sub_;
        ros::Subscriber ballisholding_sub_;
        //    ros::ServiceClient ballhandle_client_;
        //    ros::ServiceClient shoot_client_;

        ros::Publisher motor_cmd_pub_;
        ros::Publisher strategy_info_pub_;
        ros::Publisher action_cmd_pub_;
        ros::Timer control_timer_;

        boost::shared_ptr<ros::NodeHandle> nh_;

    public:
        World_Model_Info world_model_info_; /** 世界模型中的信息赋值，来源于world_model节点的topic*/
        Strategy *m_strategy_;
        Plan m_plan_;
        StaticPass m_staticpass_;

        double kp_;
        double kalpha_;
        double kbeta_;
        char match_mode_;
        char pre_match_mode_;
        DPoint robot_pos_;
        Angle robot_ori_;
        double robot_w;
        DPoint ball_pos_;
        DPoint ball_vel_;
        bool isactive = false; /// active role
        bool shoot_flag = false;
        int shoot_count = 0;
        nubot_common::BallIsHolding ball_holding_;
        nubot_common::ActionCmd action_cmd_;
        nubot_common::VelCmd vel;

        std::queue<DPoint> opp_location[5];
        std::queue<DPoint> opp2b[5];
        std::queue<DPoint> vels;
        DPoint opp_next_pos;
        DPoint opp2b_next;

    public:
        NuBotControl(int argc, char **argv)
        {
            const char *environment;
            ROS_INFO("initialize control process");

#ifdef SIMULATION
            std::string robot_name = argv[1];
            std::string num = robot_name.substr(robot_name.size() - 1);
            // std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
            environment = num.c_str();
            ROS_FATAL("control_robot_name:%s", robot_name.c_str());
            nh_ = boost::make_shared<ros::NodeHandle>(robot_name);
#else
            nh_ = boost::make_shared<ros::NodeHandle>();
            // 读取机器人标号，并赋值. 在 .bashrc 中输入export AGENT=1，2，3，4，等等；
            if ((environment = getenv("AGENT")) == NULL)
            {
                ROS_ERROR("this agent number is not read by robot");
                return;
            }
#endif
            motor_cmd_pub_ = nh_->advertise<nubot_common::VelCmd>("nubotcontrol/velcmd", 1);
            strategy_info_pub_ = nh_->advertise<nubot_common::StrategyInfo>("nubotcontrol/strategy", 10);
            action_cmd_pub_ = nh_->advertise<nubot_common::ActionCmd>("nubotcontrol/actioncmd", 1);

            //        std::string  service = "BallHandle";
            //        ballhandle_client_ =  nh_->serviceClient<nubot_common::BallHandle>(service);
            //        std::string  service1 = "Shoot";
            //        shoot_client_ = nh_->serviceClient<nubot_common::Shoot>(service1);
            worldmodelinfo_sub_ = nh_->subscribe("worldmodel/worldmodelinfo", 1, &NuBotControl::update_world_model_info, this);
            ballisholding_sub_ = nh_->subscribe("ballisholding/BallIsHolding", 1, &NuBotControl::update_ballisholding, this);
            // ballinfo3d_sub1_    = nh_->subscribe("kinect/ballinfo",1, &NuBotControl::ballInfo3dCallback, this);
            control_timer_ = nh_->createTimer(ros::Duration(0.015), &NuBotControl::loopControl, this);
            world_model_info_.AgentID_ = atoi(environment); /** 机器人标号*/
            world_model_info_.CoachInfo_.MatchMode = STOPROBOT;
            m_plan_.world_model_ = &world_model_info_;
            m_plan_.m_subtargets_.world_model_ = &world_model_info_;
            m_staticpass_.world_model_ = &world_model_info_;
            m_strategy_ = new Strategy(world_model_info_, m_plan_);
            ball_holding_.BallIsHolding = 0;
            action_cmd_.maxvel = MAXVEL;
            action_cmd_.maxw = MAXW;
            action_cmd_.move_action = No_Action;
            action_cmd_.rotate_acton = No_Action;
            action_cmd_.rotate_mode = 1;
            action_cmd_.shootPos = 0;
            action_cmd_.strength = 0;
            action_cmd_.handle_enable = 0;
        }

        ~NuBotControl()
        {
            m_plan_.m_behaviour_.app_vx_ = 0;
            m_plan_.m_behaviour_.app_vy_ = 0;
            m_plan_.m_behaviour_.app_w_ = 0;
            ball_holding_.BallIsHolding = 0;
            action_cmd_.shootPos = 0;
            action_cmd_.strength = 0;
            action_cmd_.handle_enable = 0;
            action_cmd_pub_.publish(action_cmd_);
        }

        void
        update_world_model_info(const nubot_common::WorldModelInfo &_world_msg)
        {
            /** 更新PathPlan自身与队友的信息，自身的策略信息记住最好不要更新，因为本身策略是从此传过去的*/
            for (std::size_t i = 0; i < OUR_TEAM; i++)
            {
                world_model_info_.RobotInfo_[i].setID(_world_msg.robotinfo[i].AgentID);
                world_model_info_.RobotInfo_[i].setTargetNum(1, _world_msg.robotinfo[i].targetNum1);
                world_model_info_.RobotInfo_[i].setTargetNum(2, _world_msg.robotinfo[i].targetNum2);
                world_model_info_.RobotInfo_[i].setTargetNum(3, _world_msg.robotinfo[i].targetNum3);
                world_model_info_.RobotInfo_[i].setTargetNum(4, _world_msg.robotinfo[i].targetNum4);
                world_model_info_.RobotInfo_[i].setpassNum(_world_msg.robotinfo[i].staticpassNum);
                world_model_info_.RobotInfo_[i].setcatchNum(_world_msg.robotinfo[i].staticcatchNum);

                world_model_info_.RobotInfo_[i].setLocation(DPoint(_world_msg.robotinfo[i].pos.x,
                                                                   _world_msg.robotinfo[i].pos.y));
                world_model_info_.RobotInfo_[i].setHead(Angle(_world_msg.robotinfo[i].heading.theta));
                world_model_info_.RobotInfo_[i].setVelocity(DPoint(_world_msg.robotinfo[i].vtrans.x,
                                                                   _world_msg.robotinfo[i].vtrans.y));
                world_model_info_.RobotInfo_[i].setStuck(_world_msg.robotinfo[i].isstuck);
                world_model_info_.RobotInfo_[i].setKick(_world_msg.robotinfo[i].iskick);
                world_model_info_.RobotInfo_[i].setValid(_world_msg.robotinfo[i].isvalid);
                world_model_info_.RobotInfo_[i].setW(_world_msg.robotinfo[i].vrot);
                /** 信息是来源于队友，则要更新机器人策略信息*/
                //            if(world_model_info_.AgentID_ != i+1)
                //            {
                world_model_info_.RobotInfo_[i].setDribbleState(_world_msg.robotinfo[i].isdribble);
                world_model_info_.RobotInfo_[i].setRolePreserveTime(_world_msg.robotinfo[i].role_time);
                world_model_info_.RobotInfo_[i].setCurrentRole(_world_msg.robotinfo[i].current_role);
                world_model_info_.RobotInfo_[i].setTarget(DPoint(_world_msg.robotinfo[i].target.x, _world_msg.robotinfo[i].target.y));
                //            }
            }
            /** 更新障碍物信息*/
            world_model_info_.Obstacles_.clear();
            for (nubot_common::Point2d point : _world_msg.obstacleinfo.pos)
                world_model_info_.Obstacles_.push_back(DPoint(point.x, point.y));
            //        std::cout<<"obstacles "<<world_model_info_.Obstacles_.size()<<"  "<<world_model_info_.AgentID_<<std::endl;
            world_model_info_.Opponents_.clear();
            for (nubot_common::Point2d point : _world_msg.oppinfo.pos)
                world_model_info_.Opponents_.push_back(DPoint(point.x, point.y));
            ///        std::cout<<"opponents "<<world_model_info_.Opponents_.size()<<"  "<<world_model_info_.AgentID_<<std::endl;
            /** 更新足球物信息*/
            for (std::size_t i = 0; i < OUR_TEAM; i++)
            {
                world_model_info_.BallInfo_[i].setGlobalLocation(DPoint(_world_msg.ballinfo[i].pos.x, _world_msg.ballinfo[i].pos.y));
                world_model_info_.BallInfo_[i].setRealLocation(PPoint(Angle(_world_msg.ballinfo[i].real_pos.angle),
                                                                      _world_msg.ballinfo[i].real_pos.radius));
                world_model_info_.BallInfo_[i].setVelocity(DPoint(_world_msg.ballinfo[i].velocity.x, _world_msg.ballinfo[i].velocity.y));
                world_model_info_.BallInfo_[i].setVelocityKnown(_world_msg.ballinfo[i].velocity_known);
                world_model_info_.BallInfo_[i].setLocationKnown(_world_msg.ballinfo[i].pos_known);
                world_model_info_.BallInfo_[i].setValid(_world_msg.ballinfo[i].pos_known);
            }
            world_model_info_.BallInfoState_ = _world_msg.ballinfo[world_model_info_.AgentID_ - 1].ballinfostate;

            /** 更新的COACH信息*/
            world_model_info_.CoachInfo_.MatchMode = _world_msg.coachinfo.MatchMode;
            world_model_info_.CoachInfo_.MatchType = _world_msg.coachinfo.MatchType;

            /** 更新传球信息*/
            world_model_info_.pass_cmds_.catchrobot_id = _world_msg.pass_cmd.catch_id;
            world_model_info_.pass_cmds_.passrobot_id = _world_msg.pass_cmd.pass_id;
            world_model_info_.pass_cmds_.isvalid = _world_msg.pass_cmd.is_valid;
            world_model_info_.pass_cmds_.is_dynamic_pass = _world_msg.pass_cmd.is_dynamic_pass;
            world_model_info_.pass_cmds_.is_static_pass = _world_msg.pass_cmd.is_static_pass;
            world_model_info_.pass_cmds_.is_passout = _world_msg.pass_cmd.is_passout;
            world_model_info_.pass_cmds_.pass_pt = DPoint(_world_msg.pass_cmd.pass_pt.x, _world_msg.pass_cmd.pass_pt.y);
            world_model_info_.pass_cmds_.catch_pt = DPoint(_world_msg.pass_cmd.catch_pt.x, _world_msg.pass_cmd.catch_pt.y);

            /** 这个先如此改，之后将所有数据用world_model_进行传递*/
            m_strategy_->goalie_strategy_.robot_info_ = _world_msg.robotinfo[world_model_info_.AgentID_ - 1];
            m_strategy_->goalie_strategy_.ball_info_2d_ = _world_msg.ballinfo[world_model_info_.AgentID_ - 1];
        }

        void
        update_ballisholding(const nubot_common::BallIsHolding &ball_holding)
        {
            ball_holding_.BallIsHolding = ball_holding.BallIsHolding;
        }

        /** 球的三维信息,用于守门员角色*/
        void
        ballInfo3dCallback(const nubot_common::BallInfo3d &_BallInfo_3d)
        {
            // m_strategy_->goalie_strategy_.setBallInfo3dRel( _BallInfo_3d );
        }
        /** 主要的控制框架位于这里*/
        void
        loopControl(const ros::TimerEvent &event)
        {
            match_mode_ = world_model_info_.CoachInfo_.MatchMode;     //! 当前比赛模式
            pre_match_mode_ = world_model_info_.CoachInfo_.MatchType; //! 上一个比赛模式
            robot_pos_ = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getLocation();
            robot_ori_ = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getHead();
            ball_pos_ = world_model_info_.BallInfo_[world_model_info_.AgentID_ - 1].getGlobalLocation();
            ball_vel_ = world_model_info_.BallInfo_[world_model_info_.AgentID_ - 1].getVelocity();

            // add

            // init
            m_plan_.action = &action_cmd_;
            m_plan_.robot_pos_ = robot_pos_;
            m_plan_.robot_ori_ = robot_ori_;
            m_plan_.ball_pos_ = ball_pos_;
            m_plan_.ball_vel_ = ball_vel_;
            m_plan_.m_behaviour_.action = &action_cmd_;
            m_plan_.m_behaviour_.robot_pos_ = robot_pos_;
            m_plan_.m_behaviour_.Obstacles_ = world_model_info_.Obstacles_;

            world_model_info_.update(ball_holding_.BallIsHolding);

            if (match_mode_ == STOPROBOT)
            {
                /// 运动参数
                action_cmd_.move_action = No_Action;
                action_cmd_.rotate_acton = No_Action;
            }
            /** 机器人在开始之前的跑位. 开始静态传接球的目标点计算*/
            else if (match_mode_ > STOPROBOT && match_mode_ <= DROPBALL)
                positioning();
            else if (match_mode_ == PARKINGROBOT)
                parking();
            else // 机器人正式比赛了，进入start之后的机器人状态
            {
                normalGame();
            } // start部分结束
            handleball();
            setEthercatCommand();
            pubStrategyInfo(); // 发送策略消息让其他机器人看到，这一部分一般用于多机器人之间的协同
        }

        class goalKeeper
        {
        public:
            goalKeeper() : world_(), rb_pos(), rb_ori(), ball_pos() {}
            //重载() 初始化
            void operator()(const World_Model_Info &w, const DPoint &rp, const Angle &ro, const DPoint &bp, const DPoint &bv);
            DPoint position();

        private:
            World_Model_Info world_;
            DPoint rb_pos;
            Angle rb_ori;
            DPoint ball_pos;
            DPoint ball_v;
            int getArea();
            DPoint bisector(); //角平分线站位
        };

        goalKeeper gk;

        class _pid
        {
        public:
            _pid() : kp(0.0), ki(0.6), kd(0.4) {}
            double getki() { return ki; }
            double getkd() { return kd; }
            virtual double getkp(double err) = 0;

        protected:
            double kp, ki, kd;
        };

        class Movepid : public _pid
        {
        public:
            double getkp(double err)
            {
                if (err >= 1500.0)
                    kp = 8.0;
                else if (err >= 1000)
                    kp = 7.5 + (err - 1000) * 0.001;
                else if (err > 500)
                    kp = 5.0 + (err - 500) * 0.005;
                else
                    kp = 5.0;
                return kp;
            }
        };

        class Rotatepid : public _pid
        {
        public:
            double getkp(double err)
            {
                err *= (180.0 / SINGLEPI_CONSTANT);
                if (err > 120.0)
                    kp = 6.0;
                else if (err > 60.0)
                    kp = 4.5 + (err - 60) * 0.025;
                else if (err > 30.0)
                    kp = 3.0 + (err - 30) * 0.05;
                else
                    kp = 3.0;
                return kp;
            }
        };

        int findNearestRobot()
        {
            double min_distance = 3000.0;
            auto distance = min_distance;
            int robot_id = -1;
            int i;
            for (i = 1; i < OUR_TEAM; i++)
            {
                // 排除守门员
                if (world_model_info_.RobotInfo_[i].isValid())
                {
                    distance = ball_pos_.distance(world_model_info_.RobotInfo_[i].getLocation());
                    //排除自己
                    if (i != world_model_info_.AgentID_ - 1 && distance < min_distance)
                    {
                        min_distance = distance;
                        robot_id = i;
                    }
                }
            }
            return robot_id;
        }

        void positioning()
        {
            switch (match_mode_)
            {
            case OUR_KICKOFF:
                // OurkickoffReady_();
                OurDefaultReady_();
                break;
            case OPP_KICKOFF:
                // OppkickoffReady_();
                OppDefaultReady_();
                break;
            case OUR_FREEKICK:
                OurDefaultReady_();
                break;
            case OPP_FREEKICK:
                // OppDefaultReady_();
                OppDefaultReady_();
                break;
            case OUR_GOALKICK:
                OurDefaultReady_();
                break;
            case OPP_GOALKICK:
                // OppDefaultReady_();
                OppDefaultReady_();
                break;
            case OUR_CORNERKICK:
                OurDefaultReady_();
                break;
            case OPP_CORNERKICK:
                // OppDefaultReady_();
                OppDefaultReady_();
                break;
            case OUR_THROWIN:
                OurDefaultReady_();
                break;
            case OPP_THROWIN:
                // OppDefaultReady_();
                OppDefaultReady_();
                break;
            case OUR_PENALTY:
                // OurPenaltyReady_();
                OurDefaultReady_();
                break;
            case OPP_PENALTY:
                // OppPenaltyReady_();
                OppDefaultReady_();
                break;
            case DROPBALL:
                // DropBallReady_();
                OurDefaultReady_();
                break;
            default:
                break;
            }
        }

        void OppDefaultReady_()
        {
            DPoint target;
            DPoint br = ball_pos_ - robot_pos_;
            switch (world_model_info_.AgentID_) // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
            {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
            case 1:
                target = DPoint(-1050.0, 0.0);
                break;
            case 2:
                target = DPoint(-200.0, 100.0);
                break;
            case 3:
                target = DPoint(-200.0, -100.0);
                break;
            case 4:
                target = DPoint(-550.0, 200.0);
                break;
            case 5:
                target = DPoint(-550.0, -200.0);
                break;
            }
            //不能小于3m
            if (target.distance(ball_pos_) < 300 && !world_model_info_.field_info_.isOurPenalty(target))
                target = ball_pos_.pointofline(target, 320.0);
            if (move2target(target, robot_pos_))
                move2ori(br.angle().radian_, robot_ori_.radian_);
            action_cmd_.move_action = Positioned_Static;
            action_cmd_.rotate_acton = Positioned_Static;
            action_cmd_.rotate_mode = 0;
        }
        void OurDefaultReady_()
        {
            DPoint br = ball_pos_ - robot_pos_;
            DPoint target;
            switch (world_model_info_.AgentID_) // 十分简单的实现，固定的站位，建议动态调整站位，写入staticpass.cpp中
            {                                   // 站位还需要考虑是否犯规，但是现在这个程序没有考虑。
            case 1:
                target = DPoint(-1050.0, 0.0);
                break;
            case 2:
                target = DPoint(-600, 300);
                break;
            case 3:
                target = ball_pos_.pointofline(robot_pos_, 200.0);
                break;
            case 4:
                target = DPoint(-550.0, 200.0);
                break;
            case 5:
                target = DPoint(-550.0, -200.0);
                break;
            }
            if (move2target(target, robot_pos_))
                move2ori(br.angle().radian_, robot_ori_.radian_);
            action_cmd_.move_action = Positioned_Static;
            action_cmd_.rotate_acton = Positioned_Static;
            action_cmd_.rotate_mode = 0;
        }

        void parking()
        {
            static double parking_y = -680.0;
            cout << "PARKINGROBOT" << endl;
            DPoint parking_target;
            float tar_ori = SINGLEPI_CONSTANT / 2.0;
            parking_target.x_ = FIELD_XLINE7 + 150.0 * world_model_info_.AgentID_;
            //        if(world_model_info_.AgentID_ == 1)
            //            parking_target.x_ = -900;//守门员站在离球门最近的地方
            parking_target.y_ = parking_y;

            if (move2target(parking_target, robot_pos_)) //停到目标点10cm附近就不用动了，只需调整朝向
                move2ori(tar_ori, robot_ori_.radian_);
            action_cmd_.move_action = Positioned_Static;
            action_cmd_.rotate_acton = Positioned_Static;
            action_cmd_.rotate_mode = 0;
        }

        void handleball()
        {
            if (isactive && match_mode_ == STARTROBOT && !shoot_flag)
                action_cmd_.handle_enable = 1;
            else
                action_cmd_.handle_enable = 0;
        }

        // add 4.16

        void goal_keeper()
        {
            //根据球位置获取站位
            DPoint target = gk.position();
            DPoint t2r = target - robot_pos_;
            auto b2r = ball_pos_ - robot_pos_;

            if (ball_is_free() && robot_pos_.distance(ball_pos_) <= 55)
            {
                action_cmd_.handle_enable = 1;
                action_cmd_.move_action = CatchBall;
                action_cmd_.rotate_acton = CatchBall;
                action_cmd_.rotate_mode = 0;
                if (m_plan_.m_behaviour_.move2orif(b2r.angle().radian_, robot_ori_.radian_))
                    m_plan_.m_behaviour_.move2targetFast(ball_pos_, robot_pos_);
            }

            if (!ball_is_free() && !world_model_info_.RobotInfo_[0].getDribbleState())
            {
                if (m_plan_.m_behaviour_.move2targetFast(target, robot_pos_))
                {
                    mv2orif_e1(b2r.angle().radian_, robot_ori_.radian_);
                }
            }

            else if (ball_is_free())
            {
                if (ball_vel_.x_ < 0)
                {
                    //运动到入网点
                    double k1 = ball_vel_.y_ / ball_vel_.x_;
                    double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
                    double target_y = k1 * (-1050.0) + b1;
                    if (fabs(target_y) < 135)
                    {
                        action_cmd_.handle_enable = 1;
                        action_cmd_.move_action = CatchBall;
                        action_cmd_.rotate_acton = CatchBall;
                        action_cmd_.rotate_mode = 0;
                        m_plan_.m_behaviour_.move2targetFast(DPoint(-1050.0, target_y), robot_pos_);
                        m_plan_.m_behaviour_.move2orif(b2r.angle().radian_, robot_ori_.radian_);
                    }
                    // double k2 = -1.0 / k1;
                    // double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
                    // DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
                }
            }

            if (world_model_info_.RobotInfo_[0].getDribbleState())
            {
                int pass__mode = RUN;
                auto pass_vec_ = world_model_info_.RobotInfo_[2].getLocation() - robot_pos_;

                if (pass_vec_.length() > 650)
                    pass__mode = FLY;

                shoot_flag = m_plan_.PassBall_Action(3, pass__mode);
            }
        }
        // end

        //获取端点
        inline DPoint shootPot() noexcept
        {
            // auto lot = ;
            // double m_y = lot.y_;

            //系数k
            // int k = ((m_y > 0) ? -1 : 1);

            if (world_model_info_.RobotInfo_[0].getLocation().y_ > 0)
                return world_model_info_.field_info_.ourGoal_[GOAL_MIDLOWER];
            else
                return world_model_info_.field_info_.ourGoal_[GOAL_MIDUPPER];

            // if (fabs(m_y) <= 20)
            // {
            //     d2 = DPoint(-1100.0, k * 60.0);
            // }

            // else if (fabs(m_y) < 60)
            // {
            //     d2 = DPoint(-1100.0, m_y + 120.0 * k);
            // }

            // else
            // {
            //     d2 = DPoint(-1100.0, 75.0 * k);
            // }
        }

        void update_vels()
        {
            vels.push(ball_vel_);
            while (vels.size() > 5)
                vels.pop();
        }

        void update_opp2b()
        {
            for (int i = 0; i < 5; i++)
            {
                opp2b[i].push(ball_pos_ - world_model_info_.Opponents_[i]);
                if (opp2b[i].size() > 10)
                    opp2b[i].pop();
            }
        }

        void update_opp_location()
        {
            for (int i = 0; i < 5; i++)
            {
                opp_location[i].push(world_model_info_.Opponents_[i]);
                if (opp_location[i].size() > 6)
                    opp_location[i].pop();
            }
        }

        DPoint get_opp_vel(int i)
        {
            if (opp_location[i].size() < 6)
                return DPoint(0.0, 0.0);
            double vx(0.0), vy(0.0);
            auto qlocation = opp_location[i];
            int t = 0;
            DPoint opp_pos[6], s;
            while (!qlocation.empty())
            {
                opp_pos[t++] = qlocation.front();
                qlocation.pop();
            }
            for (int i = 0; i < 3; ++i)
            {
                vx += (opp_pos[i + 3].x_ - opp_pos[i].x_) / 3;
                vy += (opp_pos[i + 3].y_ - opp_pos[i].y_) / 3;
            }
            return DPoint(vx / 0.09, vy / 0.09);
        }

        double get_opp_ori(int i)
        {
            if (opp2b[i].size() != 10)
                return 0;
            auto a2b = opp2b[i];
            int t = 0;
            double radian[10], delta_radian(0.0);
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

        void predict_next(int i)
        {
            DPoint opp_pos_now = world_model_info_.Opponents_[i];
            DPoint opp_vel = get_opp_vel(i);
            double a2b_radian = (ball_pos_ - opp_pos_now).angle().radian_;
            double a2b_ori = get_opp_ori(i);
            double a2b_radian_next;

            //   update  //
            opp_next_pos = opp_pos_now + 0.3 * opp_vel;
            a2b_radian_next = a2b_radian + 0.3 * a2b_ori;
            opp2b_next = DPoint(cos(a2b_radian_next), sin(a2b_radian_next));
        }

        int ballState()
        {
            if (vels.size() != 5)
                return 0;
            if ((vels.front() - vels.back()).length())
                return 1;
            return -1;
        }

        void normalGame()
        {
            // end
            static bool last_dribble = 0;
            isactive = false;

            if (world_model_info_.AgentID_ == 1)
            {
                // init
                gk(world_model_info_, robot_pos_, robot_ori_, ball_pos_, ball_vel_);
                goal_keeper();
            }

            // end
            else if (world_model_info_.AgentID_ == 2)
            {
                //选择二号为接球射门机器人
                action_cmd_.move_action = No_Action;
                action_cmd_.rotate_acton = No_Action;

                DPoint tmp(-800, 300);
                //射门目标点
                isactive = true;

                DPoint r2b = ball_pos_ - robot_pos_;
                DPoint r2t = tmp - robot_pos_;

                if (!shoot_flag)
                {
                    if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                    {
                        ROS_INFO("change::");
                        if (last_dribble == 0)
                            m_plan_.startPoint = robot_pos_;

                        // ROS_INFO("tx = %lf, ty = %lf", m_plan_.startPoint.x_, m_plan_.startPoint.y_);
                    }

                    last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();

                    if (ball_is_free() && robot_pos_.distance(ball_pos_) <= 75.0)
                    {
                        m_plan_.catchBall();
                    }

                    // 接球
                    if (world_model_info_.pass_state_.catch_id_ == world_model_info_.AgentID_)
                    {
                        m_plan_.CatchPassedBall();
                    }
                    // {
                    //     // auto dis = robot_pos_.distance(world_model_info_.RobotInfo_[world_model_info_.pass_state_.pass_id_ - 1].getLocation());

                    //     action_cmd_.rotate_acton = CatchBall;
                    //     action_cmd_.move_action = No_Action;
                    //     action_cmd_.rotate_mode = 0;
                    //     // move2target(robot_pos_, robot_pos_);
                    //     m_plan_.m_behaviour_.move2orif(r2b.angle().radian_, robot_ori_.radian_, 0.087);

                    //     DPoint headoffPoint;
                    //     //垂直运动球路径
                    //     if (ball_vel_.length())
                    //     {
                    //         action_cmd_.move_action = CatchBall;
                    //         double k1 = ball_vel_.y_ / ball_vel_.x_;
                    //         double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
                    //         double k2 = -1.0 / k1;
                    //         double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
                    //         headoffPoint = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
                    //     }

                    //     //运动到接球点
                    //     if (m_plan_.m_behaviour_.move2target(headoffPoint, robot_pos_))
                    //     {
                    //         //接球
                    //         if (robot_pos_.distance(ball_pos_) <= 250.0 && !world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState() && !world_model_info_.RobotInfo_[world_model_info_.pass_state_.pass_id_ - 1].getDribbleState())
                    //         {
                    //             action_cmd_.handle_enable = 1;
                    //             action_cmd_.move_action = CatchBall;
                    //             action_cmd_.rotate_acton = CatchBall;
                    //             action_cmd_.rotate_mode = 0;
                    //             if (mv2orif_e1(r2b.angle().radian_, robot_ori_.radian_))
                    //                 m_plan_.m_behaviour_.move2target(ball_pos_, robot_pos_);
                    //         }
                    //     }

                    //     // ?
                    //     if (whichopp_dribble() >= 0)
                    //         world_model_info_.pass_state_.reset();
                    // }

                    if (world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                    {

                        // plan to call attack_1()

                        if (robot_pos_.distance(tmp) > 25.0)
                        {
                            // m_plan_.moveBall(tmp);
                            action_cmd_.move_action = MoveWithBall;
                            action_cmd_.rotate_acton = MoveWithBall;
                            action_cmd_.rotate_mode = 0;
                            if (mv2orif_e1(r2t.angle().radian_, robot_ori_.radian_))
                                m_plan_.m_behaviour_.move2target(tmp, robot_pos_);
                        }

                        else
                        {
                            // m_plan_.shoot_1(shoot_flag);
                            action_cmd_.move_action = TurnForShoot;
                            action_cmd_.rotate_acton = TurnForShoot;
                            action_cmd_.rotate_mode = 0;
                            static DPoint shoot_target = shootPot();
                            auto shoot_line = shoot_target - robot_pos_;
                            m_plan_.m_behaviour_.move2target(tmp, robot_pos_);
                            m_plan_.m_behaviour_.move2oriFast(shoot_line.angle().radian_, robot_ori_.radian_, 0.09);

                            {
                                //求出球的运动轨迹方程
                                double t_x = robot_pos_.x_, t_y = robot_pos_.y_;
                                auto k = tan(robot_ori_.radian_);
                                double b = t_y - k * t_x;
                                //与 x = -1100 的交点
                                double y0 = -1100.0 * k + b;

                                bool b1 = robot_ori_.radian_ / DEG2RAD >= 90.0 || robot_ori_.radian_ / DEG2RAD <= -90.0;
                                if (b1 && y0 <= 95 && y0 >= -95 && fabs(y0 - world_model_info_.RobotInfo_[0].getLocation().y_) >= 66.0)
                                {
                                    action_cmd_.shootPos = RUN /*FLY*/;
                                    action_cmd_.strength = 500;
                                    shoot_flag = true;
                                    std::cout << "shoot done " << std::endl;
                                }
                                if (fabs(shoot_line.angle().radian_ - robot_ori_.radian_) <= 0.10)
                                {
                                    if (shoot_target == world_model_info_.field_info_.ourGoal_[GOAL_UPPER])
                                        shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_LOWER];
                                    else
                                        shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_UPPER];
                                }
                            }
                        }
                    }
                }
                else
                {
                    action_cmd_.move_action = No_Action;
                    action_cmd_.rotate_acton = No_Action;
                    if (shoot_flag)
                        shoot_count++;
                    if (shoot_count > 50)
                    {
                        shoot_count = 0;
                        shoot_flag = false;
                    }
                }

                // //接收者
                // if (world_model_info_.pass_state_.catch_id_ == world_model_info_.AgentID_)
                // {

                //     //吊射球
                //     DPoint target = robot_pos_;
                //     if (mv2orif_e1(b2r.angle().radian_, robot_ori_.radian_))
                //     {
                //         //已踢出
                //         if (!world_model_info_.RobotInfo_[world_model_info_.pass_state_.pass_id_ - 1].getDribbleState())
                //         {
                //             double k1 = ball_vel_.y_ / ball_vel_.x_;
                //             double b1 = ball_pos_.y_ - k1 * ball_pos_.x_;
                //             double k2 = -1.0 / k1;
                //             double b2 = robot_pos_.y_ - k2 * robot_pos_.x_;
                //             target = DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
                //             move2target(target, robot_pos_);
                //             // if (ball_pos_.distance(robot_pos_) <= 200 || !ball_is_free())
                //             //     world_model_info_.pass_state_.reset();
                //         }
                //     }
                // }
                // else
                // {
                //     if (!shoot_flag)
                //     {
                //         DPoint t2r = tmp - robot_pos_;
                //         // DPoint shoot_line = world_model_info_.field_info_.ourGoal_[GOAL_MIDDLE] - robot_pos_;
                //         if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                //             ROS_INFO("change::");
                //         last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();
                //         if (!world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                //         {
                //             action_cmd_.handle_enable = 1;
                //             if (mv2orif_e1(b2r.angle().radian_, robot_ori_.radian_))
                //                 move2target(ball_pos_, robot_pos_);
                //             action_cmd_.move_action = CatchBall;
                //             action_cmd_.rotate_acton = CatchBall;
                //             action_cmd_.rotate_mode = 0;
                //         }
                //         else if (robot_pos_.distance(tmp) > 25.0)
                //         {
                //             action_cmd_.move_action = MoveWithBall;
                //             action_cmd_.rotate_acton = MoveWithBall;
                //             action_cmd_.rotate_mode = 0;
                //             if (mv2orif_e1(t2r.angle().radian_, robot_ori_.radian_))
                //                 move2target(tmp, robot_pos_);
                //         }
                //         else
                //         {
                //             action_cmd_.move_action = TurnForShoot;
                //             action_cmd_.rotate_acton = TurnForShoot;
                //             action_cmd_.rotate_mode = 0;

                //             static DPoint shoot_target = pre_for_shoot();
                //             // DPoint mid((d1.x_ + d2.x_) / 2.0, (d1.y_ + d2.y_) / 2.0);
                //             // auto shoot_line = mid - robot_pos_;
                //             auto shoot_line = shoot_target - robot_pos_;
                //             move2target(tmp, robot_pos_);
                //             mv2orif_e1(shoot_line.angle().radian_, robot_ori_.radian_);

                //             {
                //                 // double low_radian_ = (d1 - robot_pos_).angle().radian_;
                //                 // double up_radian_ = (d2 - robot_pos_).angle().radian_;
                //                 // if (low_radian_ > up_radian_)
                //                 //     std::swap(low_radian_, up_radian_);

                //                 //求出球的运动轨迹方程
                //                 double t_x = robot_pos_.x_, t_y = robot_pos_.y_;
                //                 auto k = tan(robot_ori_.radian_);
                //                 double b = t_y - k * t_x;
                //                 //与 x = -1100 的交点
                //                 double y0 = -1100.0 * k + b;

                //                 bool b1 = robot_ori_.radian_ / DEG2RAD >= 90.0 || robot_ori_.radian_ / DEG2RAD <= -90.0;
                //                 if (b1 && y0 <= 95 && y0 >= -95 && fabs(y0 - world_model_info_.RobotInfo_[0].getLocation().y_) >= 66.0)
                //                 {
                //                     action_cmd_.shootPos = RUN /*FLY*/;
                //                     action_cmd_.strength = shoot_line.length() / 15;
                //                     if (action_cmd_.strength < 10.0)
                //                         action_cmd_.strength = 10.0;
                //                     shoot_flag = true;
                //                     std::cout << "shoot done " << std::endl;
                //                 }
                //                 if (fabs(shoot_line.angle().radian_ - robot_ori_.radian_) <= 0.10)
                //                 {
                //                     if (shoot_target == world_model_info_.field_info_.ourGoal_[GOAL_UPPER])
                //                         shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_LOWER];
                //                     else
                //                         shoot_target = world_model_info_.field_info_.ourGoal_[GOAL_UPPER];
                //                 }
                //             }
                //         }
                //     }

                //     else
                //     {
                //         action_cmd_.move_action = No_Action;
                //         action_cmd_.rotate_acton = No_Action;
                //         if (shoot_flag)
                //             shoot_count++;
                //         if (shoot_count > 20)
                //         {
                //             shoot_count = 0;
                //             shoot_flag = false;
                //         }
                //     }
                // }
            }

            else if (world_model_info_.AgentID_ == 3)
            {
                // nubot3 test pass ball
                isactive = true;
                // catch ball
                if (!shoot_flag)
                {
                    DPoint r2b = ball_pos_ - robot_pos_;
                    if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                    {
                        ROS_INFO("change::");
                        if (last_dribble == 0)
                            m_plan_.startPoint = robot_pos_;
                    }

                    if (world_model_info_.pass_state_.catch_id_ == world_model_info_.AgentID_)
                    {
                        ROS_INFO("receive");
                        m_plan_.CatchPassedBall();
                    }

                    last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();

                    if (ball_is_free() && !last_dribble)
                    {
                        action_cmd_.handle_enable = 1;
                        if (m_plan_.m_behaviour_.move2oriFast(r2b.angle().radian_, robot_ori_.radian_, 0.12))
                            m_plan_.m_behaviour_.move2target(ball_pos_, robot_pos_);
                        action_cmd_.move_action = CatchBall;
                        action_cmd_.rotate_acton = CatchBall;
                        action_cmd_.rotate_mode = 0;
                    }
                    // passball
                    else if (last_dribble)
                    {
                        int pass__mode = RUN;
                        auto pass_vec_ = world_model_info_.RobotInfo_[1].getLocation() - robot_pos_;

                        if (pass_vec_.length() > 650)
                            pass__mode = FLY;

                        shoot_flag = m_plan_.PassBall_Action(2, pass__mode);
                    }
                }
                else
                {
                    action_cmd_.move_action = No_Action;
                    action_cmd_.rotate_acton = No_Action;
                    if (shoot_flag)
                        shoot_count++;
                    if (shoot_count > 60)
                    {
                        shoot_count = 0;
                        shoot_flag = false;
                    }
                }
            }

            else
            {
                m_plan_.m_behaviour_.selfRotate(robot_ori_.radian_);
                // if (world_model_info_.AgentID_ != 1 && isNearestRobot())
                // {
                //     isactive = true;
                // }
                // if (isactive && !shoot_flag)
                // {
                //     DPoint b2r = ball_pos_ - robot_pos_;
                //     DPoint tmp(500.0, 200.0);
                //     DPoint t2r = tmp - robot_pos_;
                //     DPoint shoot_line = world_model_info_.field_info_.oppGoal_[GOAL_MIDDLE] - robot_pos_;
                //     if (last_dribble != world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                //     {
                //         if (last_dribble == 0)
                //             m_plan_.startPoint = robot_pos_;
                //         ROS_INFO("change::");
                //     }

                //     last_dribble = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState();

                //     if (!world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getDribbleState())
                //     {
                //         action_cmd_.handle_enable = 1;
                //         if (move2ori(b2r.angle().radian_, robot_ori_.radian_))
                //             move2target(ball_pos_, robot_pos_);
                //         action_cmd_.move_action = CatchBall;
                //         action_cmd_.rotate_acton = CatchBall;
                //         action_cmd_.rotate_mode = 0;
                //     }
                //     else if (robot_pos_.distance(tmp) > 30.0)
                //     {
                //         action_cmd_.move_action = MoveWithBall;
                //         action_cmd_.rotate_acton = MoveWithBall;
                //         action_cmd_.rotate_mode = 0;
                //         if (move2ori(t2r.angle().radian_, robot_ori_.radian_))
                //             move2target(tmp, robot_pos_);
                //     }
                //     else
                //     {
                //         action_cmd_.move_action = TurnForShoot;
                //         action_cmd_.rotate_acton = TurnForShoot;
                //         action_cmd_.rotate_mode = 0;
                //         move2target(tmp, robot_pos_);
                //         move2ori(shoot_line.angle().radian_, robot_ori_.radian_);

                //         {
                //             double up_radian_ = (world_model_info_.field_info_.oppGoal_[GOAL_MIDUPPER] - robot_pos_).angle().radian_;
                //             double low_radian_ = (world_model_info_.field_info_.oppGoal_[GOAL_MIDLOWER] - robot_pos_).angle().radian_;
                //             if (robot_ori_.radian_ > low_radian_ && robot_ori_.radian_ < up_radian_)
                //             {
                //                 action_cmd_.shootPos = RUN /*FLY*/;
                //                 action_cmd_.strength = shoot_line.length() / 25.0;
                //                 if (action_cmd_.strength < 15.0)
                //                     action_cmd_.strength = 15.0;
                //                 shoot_flag = true;
                //                 std::cout << "shoot done " << std::endl;
                //             }
                //         }
                //     }
                // }
                // else
                // {
                //     action_cmd_.move_action = No_Action;
                //     action_cmd_.rotate_acton = No_Action;
                //     if (shoot_flag)
                //         shoot_count++;
                //     if (shoot_count > 20)
                //     {
                //         shoot_count = 0;
                //         shoot_flag = false;
                //     }
                // }
            }
        }

        // avoid
        const double radius_robot = 50.0, radius_Obs = 50.0;
        DPoint subtargets_pos_;

        void _min(int n, double *nums, int &index, double &val)
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

        void _max(int n, double *nums, int &index, double &val)
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

        void subtarget(DPoint &target_pos_, DPoint &robot_pos_)
        {
            std::vector<DPoint> Obstacles_ = world_model_info_.Obstacles_;
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
                subtargets_pos_ = target_pos_;
        }

        void relocate(int obs_num, double *cos_cast, double *sin_cast,
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

            subtargets_pos_.x_ = robot_pos_.x_ + (cos(alpha_val) * r2t.x_ -
                                                  sin(alpha_val) * r2t.y_) *
                                                     Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / r2t.length();
            subtargets_pos_.y_ = robot_pos_.y_ +
                                 (sin(alpha_val) * r2t.x_ + cos(alpha_val) * r2t.y_) * Obstacles_.at(obs_group[alpha_id]).distance(robot_pos_) / r2t.length();
        }
        // avoid end

        //对方几号带球, -1
        int whichopp_dribble()
        {
            for (int i = 0; i < 5; ++i)
                if (world_model_info_.Opponents_[i].distance(ball_pos_) < 36)
                    return i;
            return -1;
        }

        //球是否自由运动
        bool ball_is_free()
        {
            for (int i = 0; i < 5; ++i)
            {
                if (world_model_info_.RobotInfo_[i].getDribbleState())
                    return false;
            }
            if (whichopp_dribble() >= 0)
                return false;
            return true;
        }

        bool isNearestRobot() //判断自己是否为离球最近的机器人
        {
            float distance_min = 2000.0;
            float distance = distance_min;
            int robot_id = -1;

            for (int i = 1; i < OUR_TEAM; i++) // 排除守门员
                if (world_model_info_.RobotInfo_[i].isValid())
                {
                    distance = ball_pos_.distance(world_model_info_.RobotInfo_[i].getLocation());
                    if (distance < distance_min)
                    {
                        distance_min = distance;
                        robot_id = i;
                    }
                }
            if (robot_id + 1 == world_model_info_.AgentID_)
                return true;
            else
                return false;
        }

        //
        bool move2target(DPoint target, DPoint pos, double distance_thres = 20.0)
        {
            subtarget(target, robot_pos_);
            target = subtargets_pos_;

            shared_ptr<_pid> moveControl = make_shared<Movepid>();
            // static _pid *moveControl = new Movepid;
            static int cnt = 0;
            static double sum(0);
            if (cnt >= 50)
            {
                cnt = 0;
                sum = 0;
            }
            static double err;
            static double pre_err = pos.distance(target);
            err = pos.distance(target);
            //积分
            sum += err;
            ++cnt;
            auto ierr = sum / cnt;

            //微分
            double derr = (err - pre_err);

            action_cmd_.target.x = target.x_;
            action_cmd_.target.y = target.y_;
            action_cmd_.maxvel = moveControl->getkp(err) * err +
                                 moveControl->getki() * ierr + moveControl->getkd() * derr;
            pre_err = err;
            if (pos.distance(target) > distance_thres)
                return false;
            else
                return true;
        }

        //
        bool move2ori(double target, double angle, double angle_thres = 8.0 * DEG2RAD)
        {
            if (fabs(target) / DEG2RAD >= 173.0)
            {
                target = fabs(target);
                angle = fabs(angle);
            }
            shared_ptr<_pid> rotateControl = make_shared<Rotatepid>();
            // static _pid *rotateControl = new Rotatepid;
            static int cnt = 0;
            static double sum(0);
            if (cnt >= 50)
            {
                cnt = 0;
                sum = 0;
            }
            static double err;
            static double pre_err = fabs(target - angle);
            err = fabs(target - angle);
            //积分
            sum += err;
            ++cnt;
            double ierr = sum / cnt;

            //微分
            double derr = (err - pre_err);

            pre_err = err;
            action_cmd_.target_ori = target;
            action_cmd_.maxw = rotateControl->getkp(err) * err +
                               rotateControl->getki() * ierr + rotateControl->getkd() * derr;
            if (err > angle_thres) // 容许误差为5度
                return false;
            else
                return true;
        }
        // PD控制
        bool mv2orif_e1(double target, double angle, double angle_thres = 8.0 * DEG2RAD)
        {
            if (fabs(target) / DEG2RAD >= 173.0)
            {
                target = fabs(target);
                angle = fabs(angle);
            }
            shared_ptr<_pid> rotateControl = make_shared<Rotatepid>();
            static double err;
            static double pre_err = fabs(target - angle);
            err = fabs(target - angle);

            //微分d
            double derr = (err - pre_err);

            pre_err = err;
            action_cmd_.target_ori = target;
            action_cmd_.maxw = (rotateControl->getkp(err) + 0.8) * err + (rotateControl->getkd() - 0.1) * derr;
            if (err > angle_thres)
                return false;
            else
                return true;
        }

        void setEthercatCommand()
        {
            /// initialize the command
            nubot_common::ActionCmd command;
            command.move_action = No_Action;
            command.rotate_acton = No_Action;
            command.rotate_mode = 0;
            command.maxvel = 0;
            command.maxw = 0;
            command.target_w = 0;
            /// 机器人人位置信息 robot states
            command.robot_pos.x = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getLocation().x_;
            command.robot_pos.y = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getLocation().y_;
            command.robot_vel.x = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getVelocity().x_;
            command.robot_vel.y = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getVelocity().y_;
            command.robot_ori = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getHead().radian_;
            command.robot_w = world_model_info_.RobotInfo_[world_model_info_.AgentID_ - 1].getW();
            /// 运动参数
            command.move_action = action_cmd_.move_action;
            command.rotate_acton = action_cmd_.rotate_acton;
            command.rotate_mode = action_cmd_.rotate_mode;
            command.target = action_cmd_.target;
            command.target_vel = action_cmd_.target_vel;
            command.target_w = action_cmd_.target_w;
            command.target_ori = action_cmd_.target_ori;
            command.maxvel = action_cmd_.maxvel;
            command.maxw = action_cmd_.maxw;
            if (command.maxvel > MAXVEL)
                command.maxvel = MAXVEL;
            if (command.maxw > MAXW)
                command.maxw = MAXW;
            if (fabs(command.target_ori) > 10000.0)
                command.target_ori = 0;
            /// 带球及射门选择
            command.handle_enable = action_cmd_.handle_enable;
            command.strength = action_cmd_.strength;
            if (command.strength != 0)
                std::cout << "passed out" << command.strength << std::endl;
            command.shootPos = action_cmd_.shootPos;
            /// 传一次后，力量清0,防止多次射门
            action_cmd_.strength = 0;
            action_cmd_pub_.publish(command);
        }
        void pubStrategyInfo()
        {
            nubot_common::StrategyInfo strategy_info; // 这个消息的定义可以根据个人需要进行修改
            strategy_info.header.stamp = ros::Time::now();
            strategy_info.AgentID = world_model_info_.AgentID_;
            strategy_info.is_dribble = ball_holding_.BallIsHolding;

            // add
            strategy_info.pass_cmd.pass_id = world_model_info_.pass_ID_;
            strategy_info.pass_cmd.catch_id = world_model_info_.catch_ID_;
            strategy_info.pass_cmd.is_passout = world_model_info_.is_passed_out_;
            strategy_info.pass_cmd.is_valid = world_model_info_.pass_cmds_.isvalid;

            strategy_info.pass_cmd.is_dynamic_pass = world_model_info_.is_dynamic_pass_;
            strategy_info.pass_cmd.is_static_pass = world_model_info_.is_static_pass_;

            strategy_info.pass_cmd.pass_pt.x = world_model_info_.pass_pt_.x_;
            strategy_info.pass_cmd.pass_pt.y = world_model_info_.pass_pt_.y_;
            strategy_info.pass_cmd.catch_pt.x = world_model_info_.catch_pt_.x_;
            strategy_info.pass_cmd.catch_pt.y = world_model_info_.catch_pt_.y_;

            // bool b = ((match_mode_ == STOPROBOT) || (match_mode_ > STOPROBOT && match_mode_ <= DROPBALL) || (match_mode_ == PARKINGROBOT));

            // if (world_model_info_.AgentID_ == 3 && !b)
            //     RO("in pub : catchid %d", strategy_info.pass_cmd.catch_id);

            strategy_info_pub_.publish(strategy_info);
        }
    };
    void NuBotControl::goalKeeper::operator()(const World_Model_Info &w, const DPoint &rp, const Angle &ro, const DPoint &bp, const DPoint &bv)
    {
        world_ = w;
        rb_pos = rp;
        rb_ori = ro;
        ball_pos = bp;
        ball_v = bv;
    }
    int NuBotControl::goalKeeper::getArea()
    {
        DPoint shoot_robot = world_.RobotInfo_[1].getLocation();
        // bool _b = world_.field_info_.isOurGoal(ball_pos);
        double k = ball_pos.y_ / (ball_pos.x_ + 1100.0);
        if (k > 2.5)
            return 4;
        if (k < -2.5)
            return 5;
        DPoint mid(-1100, 0);

        double s = ball_pos.distance(mid);
        if (s < 209)
            return 1;
        if (s < 412)
            return 2;
        if (s < 750)
            return 3;
        return 6;
    }

    //平分角，平分线斜率
    inline double divk(double k1, double k2)
    {
        return ((k1 * k2 - 1 + sqrt((1 - k1 * k2) * (1 - k1 * k2) + (k1 + k2) * (k1 + k2))) / (k1 + k2));
    }

    //角平分线站位
    DPoint NuBotControl::goalKeeper::bisector()
    {
        //球与门柱上下连线斜率
        auto up_k = (ball_pos.y_ - 120) / (ball_pos.x_ + 1100.0);
        auto down_k = (ball_pos.y_ + 120) / (ball_pos.x_ + 1100.0);

        //角平分线斜率
        auto k = divk(up_k, down_k);
        //确保是内角平分线
        k = ((ball_pos.y_ > 0) ? fabs(k) : -fabs(k));
        return DPoint(-1050.0, -k * 1050.0 + ball_pos.y_ - k * ball_pos.x_);
    }

    //获取目标点
    DPoint NuBotControl::goalKeeper::position()
    {
        auto num = getArea();
        if (num == 4)
            return DPoint(-1050, 120);
        if (num == 5)
            return DPoint(-1050, -120);

        //可去
        if (num == 6)
            return DPoint(-1050, 0);

        //角平分线站位
        if (num == 3)
            return bisector();

        // if (num == 2)
        // {
        //     //球向球门移动，垂直运动到球的路径上
        //     if (ball_v.x_ < 0)
        //     {
        //         double k1 = ball_v.y_ / ball_v.x_;
        //         double b1 = ball_pos.y_ - k1 * ball_pos.x_;
        //         double k2 = -1.0 / k1;
        //         double b2 = rb_pos.y_ - k2 * rb_pos.x_;
        //         return DPoint((b2 - b1) / (k1 - k2), (k1 * b2 - k2 * b1) / (k1 - k2));
        //     }
        //     //否则采用角平分线站位//
        //     else
        //         return this->bisector();
        // }

        // num == 1 与 num == 2 num == 6 的情况
        return DPoint(-1050, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nubot_control_node");
    // 完成一系列的初始化工作？ 以及相应的报错机制。  只有当所有的传感器信息都已经准备就绪的时候才可以运行
    ros::Time::init();
    ROS_INFO("start control process");
    nubot::NuBotControl nubotcontrol(argc, argv);
    ros::spin();
    return 0;
}
