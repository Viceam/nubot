#include "nubot/nubot_control/staticpass.h"

using namespace nubot;

StaticPass::StaticPass() : num_in_field(0), out_of_field(0), opp_in_field(0), opp_out_field(0)
{
    isPosition_ = false;
    m_nCanBeInPenaltyArea = 0;
    m_nPassNumber_ = -1;
    m_nCatchNumber_ = -1;
    ballNumber_ = -1;
    ballPos_ = DPoint(0, 0);
    backFieldPoint_ = DPoint(-600, 0);
    for (int i = 0; i < OUR_TEAM; i++)
    {
        isAllocation_[i] = false;
        targetInit_[i] = DPoint(0, 0);
    }
    targetInit_[0] = DPoint(-1090, 0); //静态站位时，守门员基本位置恒定
    target_ = DPoint(0, 0);            //为分配的目标点
    m_nPassNumber_ = 0;
    m_nCatchNumber_ = 0;
    for (int i = 0; i < 4; i++)
    {
        opp_id_in[i] = 0;
        opp_id_out[i] = 0;
        ID_in_field[i] = 0;
        ID_out_field[i] = 0;
        target_pos_[i] = DPoint(0.0, 0.0);
        target_opp_pos[i] = DPoint(0.0, 0.0);
    }
}

StaticPass::~StaticPass()
{
}

void StaticPass::staticReady_() //判断何种站位
{
    switch (world_model_->CoachInfo_.MatchMode)
    {
    case OUR_KICKOFF:
        // OurkickoffReady_();
        OurDefaultReady_();
        break;
    case OPP_KICKOFF:
        // OppkickoffReady_();
        OurDefaultReady_();
        break;
    case OUR_FREEKICK:
        OurDefaultReady_();
        break;
    case OPP_FREEKICK:
        // OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_GOALKICK:
        OurDefaultReady_();
        break;
    case OPP_GOALKICK:
        // OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_CORNERKICK:
        OurDefaultReady_();
        break;
    case OPP_CORNERKICK:
        // OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_THROWIN:
        OurDefaultReady_();
        break;
    case OPP_THROWIN:
        // OppDefaultReady_();
        OurDefaultReady_();
        break;
    case OUR_PENALTY:
        // OurPenaltyReady_();
        OurDefaultReady_();
        break;
    case OPP_PENALTY:
        // OppPenaltyReady_();
        OurDefaultReady_();
        break;
    case DROPBALL:
        // DropBallReady_();
        OurDefaultReady_();
        break;
    default:
        break;
    }
}

void StaticPass::init_our_in_field()
{
    num_in_field = 0;
    out_of_field = 0;
    opp_in_field = 0;
    opp_out_field = 0;

    for (int ID = 2; ID < 6; ID += 1)
    {
        if (world_model_->field_info_.isInInterField2(world_model_->RobotInfo_[ID - 1].getLocation(), 100.0, 100.0))
        {
            ID_in_field[num_in_field] = ID;
            num_in_field++;
        }
        else
        {
            ID_out_field[out_of_field] = ID;
            out_of_field++;
        }
    }

    for (int i = 1; i < 5; i++)
    {
        if (world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0))
        {
            opp_id_in[opp_in_field] = i;
            opp_in_field++;
        }
        else
        {
            opp_id_out[opp_out_field] = i;
            opp_out_field++;
        }
    }
}

bool StaticPass::TeammatesInPenalty()
{
    for (int i = 1; i < 5; i++)
    {
        if (i != world_model_->AgentID_ - 1 && world_model_->field_info_.isOurPenalty(world_model_->RobotInfo_[i].getLocation()))
            return true;
    }
    return false;
}

void StaticPass::OurfreekickReady_() //我方任意球
{
    init_our_in_field();
    if (ballPos_.x_ < 0.0)
    {
        target_pos_[0] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ + 50.0) : DPoint(ballPos_.x_, ballPos_.y_ - 50.0);
        target_pos_[1] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_ + 300.0, ballPos_.y_ - 50.0) : DPoint(ballPos_.x_ + 300.0, ballPos_.y_ + 50.0);
        target_pos_[2] = DPoint(300.0, 400.0 * (ballPos_.y_) / fabs(ballPos_.y_));
        target_pos_[3] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 210.0) : DPoint(ballPos_.x_, ballPos_.y_ + 210.0);
    }
    else
    {
        DPoint tmp(1100.0, 0.0);
        target_pos_[0] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ + 75.0) : DPoint(ballPos_.x_, ballPos_.y_ - 75.0);
        target_pos_[1] = ballPos_ + (210.0 / (tmp - ballPos_).length()) * (tmp - ballPos_);
        target_pos_[2] = DPoint(700.0, 0.0);
        target_pos_[3] = DPoint(200.0, 0.0);
    }
    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
        if (world_model_->field_info_.isOurGoal(target_))
        {
            double l1 = 230 - target_.y_;
            double l2 = -970 - target_.x_;
            double l3 = target_.y_ + 230;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 230) * (ballPos_.y_ - 230));
                target_.y_ = 230;
            }
            if (min == l2)
            {
                target_.x_ = -970;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -230;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-230 - ballPos_.y_) * (-230 - ballPos_.y_));
            }
        }
        if (world_model_->field_info_.isOurPenalty(target_))
        {
            double l1 = 380 - target_.y_;
            double l2 = 830 - target_.x_;
            double l3 = target_.y_ + 380;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                target_.y_ = 380;
            }
            if (min == l2)
            {
                target_.x_ = 830;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -380;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
            }
        }
        if (TeammatesInPenalty())
        {
            if (world_model_->field_info_.isOurPenalty(target_))
            {
                double l1 = 380 - target_.y_;
                double l2 = -830 - target_.x_;
                double l3 = target_.y_ + 380;
                double min = l1;
                if (min > l2)
                {
                    min = l2;
                    if (min > l3)
                    {
                        min = l3;
                    }
                }
                if (min == l1)
                {
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                    target_.y_ = 380;
                }
                if (min == l2)
                {
                    target_.x_ = -830;
                    if (ballPos_.y_ >= 0)
                    {
                        target_.y_ = ballPos_.y_ - sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                    else
                    {
                        target_.y_ = ballPos_.y_ + sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                }
                if (min == l3)
                {
                    target_.y_ = -380;
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
                }
            }
        }
    }
}

void StaticPass::OppfreekickReady_() //对方任意球
{
}

void StaticPass::OurThrowinReady_()
{
    init_our_in_field();
    if (ballPos_.x_ < 0.0)
    {
        target_pos_[0] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ + 50.0) : DPoint(ballPos_.x_, ballPos_.y_ - 50.0);
        target_pos_[1] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_ + 300.0, ballPos_.y_ - 50.0) : DPoint(ballPos_.x_ + 300.0, ballPos_.y_ + 50.0);
        target_pos_[2] = DPoint(500.0, 0.0);
        target_pos_[3] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 210.0) : DPoint(ballPos_.x_, ballPos_.y_ + 210.0);
    }
    else
    {
        DPoint tmp(1100.0, 0.0);
        target_pos_[0] = (ballPos_.y_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ + 75.0) : DPoint(ballPos_.x_, ballPos_.y_ - 75.0);
        target_pos_[1] = ballPos_ + (210.0 / (tmp - ballPos_).length()) * (tmp - ballPos_);
        target_pos_[2] = DPoint(700.0, 0.0);
        target_pos_[3] = DPoint(200.0, 0.0);
    }
    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
        if (world_model_->field_info_.isOurGoal(target_))
        {
            double l1 = 230 - target_.y_;
            double l2 = -970 - target_.x_;
            double l3 = target_.y_ + 230;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 230) * (ballPos_.y_ - 230));
                target_.y_ = 230;
            }
            if (min == l2)
            {
                target_.x_ = -970;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -230;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-230 - ballPos_.y_) * (-230 - ballPos_.y_));
            }
        }
        if (TeammatesInPenalty() == true)
        {
            if (world_model_->field_info_.isOurPenalty(target_))
            {
                double l1 = 380 - target_.y_;
                double l2 = -830 - target_.x_;
                double l3 = target_.y_ + 380;
                double min = l1;
                if (min > l2)
                {
                    min = l2;
                    if (min > l3)
                    {
                        min = l3;
                    }
                }
                if (min == l1)
                {
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                    target_.y_ = 380;
                }
                if (min == l2)
                {
                    target_.x_ = -830;
                    if (ballPos_.y_ >= 0)
                    {
                        target_.y_ = ballPos_.y_ - sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                    else
                    {
                        target_.y_ = ballPos_.y_ + sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                }
                if (min == l3)
                {
                    target_.y_ = -380;
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
                }
            }
        }
    }
}

void StaticPass::OppThrowinReady_()
{
    int kickerID;

    init_our_in_field();

    DPoint target__sz[3];
    DPoint ABvector;
    DPoint tmp;
    double lengthmin = 10000.0;
    double xmax = -10000.0;
    double xmin = 10000.0;
    int num_in_ok = 0;
    int maxID, minID;
    for (int i = 1; i < 5; i++)
    {
        if ((world_model_->Opponents_[i] - ballPos_).length() < lengthmin)
        {
            kickerID = i;
            lengthmin = (world_model_->Opponents_[i] - ballPos_).length();
        }
    }
    for (int i = 1, j = 0; i < 5; i++)
    {
        if (i != kickerID && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0) && world_model_->Opponents_[i].x_ < 1000.0)
        {
            ABvector = world_model_->Opponents_[i] - ballPos_;
            if (ABvector.length() < 700 && (ABvector.length() > 370.0 || ABvector.length() < 250.0))
            {
                target__sz[j] = ballPos_ + (310.0 / ABvector.length()) * ABvector;
            }
            else if (ABvector.length() <= 370.0 && ABvector.length() >= 250.0)
            {
                target__sz[j] = world_model_->Opponents_[i] + (100.0 / ABvector.length()) * ABvector;
            }
            else
            {
                target__sz[j] = world_model_->Opponents_[i] + (-100.0 / ABvector.length()) * ABvector;
            }
            j++;
            num_in_ok++;

            // if (world_model_->AgentID_ == 1)
            //     ROS_INFO("we are %d,the target__sz[%d] = x: %lf, y: %lf", num_in_ok - 1, num_in_ok - 1, target__sz[num_in_ok - 1].x_, target__sz[num_in_ok - 1].y_);
        }
    }
    // if (world_model_->AgentID_ == 1)
    //     ROS_INFO("the num_in_ok is %d", num_in_ok);
    for (int i = num_in_ok - 2; i >= 0; i--)
    {
        for (int j = 0; j <= i; j++)
        {
            if (target__sz[j].x_ > target__sz[j + 1].x_)
            {
                tmp = target__sz[j];
                target__sz[j] = target__sz[j + 1];
                target__sz[j + 1] = tmp;
            }
        }
    }
    if (ballPos_.x_ > 0.0)
    {
        target_pos_[0] = DPoint(200.0, 400 * ballPos_.y_ / fabs(ballPos_.y_));
        target_pos_[1] = DPoint(0.0, 300 * ballPos_.y_ / fabs(ballPos_.y_));
        target_pos_[2] = DPoint(-400.0, 0.0);
        target_pos_[3] = DPoint(0.0, 0.0);
        for (int i = 0; i < num_in_ok; i++)
        {
            target_pos_[i] = target__sz[i];
        }
    }
    else
    {
        target_pos_[0] = DPoint(-800.0, 400.0 * ballPos_.y_ / fabs(ballPos_.y_));
        target_pos_[1] = DPoint(-500.0, 300 * ballPos_.y_ / fabs(ballPos_.y_));
        target_pos_[2] = DPoint(-400.0, 0.0);
        target_pos_[3] = DPoint(0.0, 200.0 * ballPos_.y_ / fabs(ballPos_.y_));
        for (int i = 0; i < num_in_ok; i++)
        {
            target_pos_[i] = target__sz[i];
        }
    }
    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
        if (world_model_->field_info_.isOppPenalty(target_))
        {
            double l1 = 380 - target_.y_;
            double l2 = 830 - target_.x_;
            double l3 = target_.y_ + 380;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                target_.y_ = 380;
            }
            if (min == l2)
            {
                target_.x_ = 830;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -380;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
            }
        }
        if (world_model_->field_info_.isOurGoal(target_))
        {
            double l1 = 230 - target_.y_;
            double l2 = -970 - target_.x_;
            double l3 = target_.y_ + 230;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 230) * (ballPos_.y_ - 230));
                target_.y_ = 230;
            }
            if (min == l2)
            {
                target_.x_ = -970;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -230;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-230 - ballPos_.y_) * (-230 - ballPos_.y_));
            }
        }
        if (TeammatesInPenalty() == true)
        {
            if (world_model_->field_info_.isOurPenalty(target_))
            {
                double l1 = 380 - target_.y_;
                double l2 = -830 - target_.x_;
                double l3 = target_.y_ + 380;
                double min = l1;
                if (min > l2)
                {
                    min = l2;
                    if (min > l3)
                    {
                        min = l3;
                    }
                }
                if (min == l1)
                {
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                    target_.y_ = 380;
                }
                if (min == l2)
                {
                    target_.x_ = -830;
                    if (ballPos_.y_ >= 0)
                    {
                        target_.y_ = ballPos_.y_ - sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                    else
                    {
                        target_.y_ = ballPos_.y_ + sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                }
                if (min == l3)
                {
                    target_.y_ = -380;
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
                }
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if (world_model_->AgentID_ == 1)
        {
            ROS_INFO("the Id is %d,target_pos_ = x: %lf ,y: %lf ", ID_in_field[i], target_pos_[i].x_, target_pos_[i].y_);
        }
    }
}

void StaticPass::OurGoalReady_() //我方Goal发球站位
{
    int flagy = -1;
    //    DPoint target_;
    if (ballPos_.y_ > 0)
        flagy = 1;
    DPoint target_;
    init_our_in_field();
    target_pos_[0] = DPoint(-800.0, ballPos_.y_);
    target_pos_[1] = DPoint(-600.0, flagy * 200.0);
    target_pos_[2] = DPoint(500.0, ballPos_.y_);
    target_pos_[3] = DPoint(-200.0, flagy * 200.0);

    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
        if (world_model_->field_info_.isOppPenalty(target_))
        {
            double l1 = 380 - target_.y_;
            double l2 = 830 - target_.x_;
            double l3 = target_.y_ + 380;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                target_.y_ = 380;
            }
            if (min == l2)
            {
                target_.x_ = 830;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -380;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
            }
        }
        if (world_model_->field_info_.isOurGoal(target_))
        {
            double l1 = 230 - target_.y_;
            double l2 = -970 - target_.x_;
            double l3 = target_.y_ + 230;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 230) * (ballPos_.y_ - 230));
                target_.y_ = 230;
            }
            if (min == l2)
            {
                target_.x_ = -970;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -230;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-230 - ballPos_.y_) * (-230 - ballPos_.y_));
            }
        }
        if (TeammatesInPenalty() == true)
        {
            if (world_model_->field_info_.isOurPenalty(target_))
            {
                double l1 = 380 - target_.y_;
                double l2 = -830 - target_.x_;
                double l3 = target_.y_ + 380;
                double min = l1;
                if (min > l2)
                {
                    min = l2;
                    if (min > l3)
                    {
                        min = l3;
                    }
                }
                if (min == l1)
                {
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                    target_.y_ = 380;
                }
                if (min == l2)
                {
                    target_.x_ = -830;
                    if (ballPos_.y_ >= 0)
                    {
                        target_.y_ = ballPos_.y_ - sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                    else
                    {
                        target_.y_ = ballPos_.y_ + sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                }
                if (min == l3)
                {
                    target_.y_ = -380;
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
                }
            }
        }
    }
}

void StaticPass::OppGoalReady_()
{
    int num_inourfield = 0;
    int kickerID;
    double lengthmin = 10000.0;
    DPoint target__sz[3];
    DPoint ABvector;
    DPoint target_;
    DPoint opp_inourfield[4];
    init_our_in_field();
    for (int i = 1; i < 5; i++)
    {
        if (world_model_->Opponents_[i].x_ < 0.0 && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0))
        {
            opp_inourfield[num_inourfield] = world_model_->Opponents_[i];
            num_inourfield++; // 记录在己方半场的对方的球员的数量和ID
        }
        if ((world_model_->Opponents_[i] - ballPos_).length() < lengthmin && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0)) //记录对方的发球人员的ID)
        {
            kickerID = i;
            lengthmin = (world_model_->Opponents_[i] - ballPos_).length();
        }
    }
    for (int i = 1, j = 0; i < 5; i++)
    { //记录在对方半场且不是发球人员的位置以及拦截位置
        if (i != kickerID && world_model_->Opponents_[i].x_ >= 0.0 && world_model_->Opponents_[i].x_ < 1000.0 && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0))
        {
            ABvector = world_model_->Opponents_[i] - ballPos_;
            target__sz[j] = ballPos_ + (310.0 / ABvector.length()) * ABvector;
            j++;
        }
    }
    if (num_inourfield == 0)
    { // opp no in our field
        target_pos_[0] = DPoint(400.0, (300.0 * world_model_->Opponents_[kickerID].y_) / fabs(world_model_->Opponents_[kickerID].y_));
        target_pos_[1] = DPoint(0.0, (200.0 * world_model_->Opponents_[kickerID].y_) / fabs(world_model_->Opponents_[kickerID].y_));
        target_pos_[2] = DPoint(400.0, 0.0);
        target_pos_[3] = DPoint(0.0, 0.0);
        for (int i = 0; i < (opp_in_field - 1); i++)
        {
            target_pos_[i] = target__sz[i];
        }
    }
    else if (num_inourfield == 1)
    { // one opp in our field
        target_pos_[0] = opp_inourfield[0] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[1] = DPoint(0.0, 200 * world_model_->Opponents_[kickerID].y_ / fabs(world_model_->Opponents_[kickerID].y_));
        target_pos_[2] = DPoint(400.0, 0.0);
        target_pos_[3] = DPoint(0.0, 0.0);
        for (int i = 0; i < (opp_in_field - 2); i++)
        {
            target_pos_[i + 1] = target__sz[i];
        }
    }
    else if (num_inourfield == 2)
    { // two opp in our field
        target_pos_[0] = opp_inourfield[0] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[1] = opp_inourfield[1] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[2] = DPoint(0.0, 200 * world_model_->Opponents_[kickerID].y_ / fabs(world_model_->Opponents_[kickerID].y_));
        target_pos_[3] = DPoint(0.0, 0.0);

        for (int i = 0; i < (opp_in_field - 3); i++)
        {
            target_pos_[i + 2] = target__sz[i];
        }
    }
    else if (num_inourfield == 3)
    { // three opp in our field
        target_pos_[0] = opp_inourfield[0] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[1] = opp_inourfield[1] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[2] = opp_inourfield[2] + (100.0 / (ballPos_ - opp_inourfield[0]).length()) * (ballPos_ - opp_inourfield[0]);
        target_pos_[3] = DPoint(0.0, 0.0);
        for (int i = 0; i < (opp_in_field - 4); i++)
        {
            target_pos_[i + 3] = target__sz[i];
        }
    }

    if (world_model_->AgentID_ == 1)
    {
        target_ = DPoint(-1090.0, 0.0);
    }
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
        if (world_model_->field_info_.isOurGoal(target_))
        {
            double l1 = 230 - target_.y_;
            double l2 = -970 - target_.x_;
            double l3 = target_.y_ + 230;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 230) * (ballPos_.y_ - 230));
                target_.y_ = 230;
            }
            if (min == l2)
            {
                target_.x_ = -970;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (-230 - ballPos_.x_) * (-230 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -230;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-230 - ballPos_.y_) * (-230 - ballPos_.y_));
            }
        }
        if (world_model_->field_info_.isOppPenalty(target_))
        {
            double l1 = 380 - target_.y_;
            double l2 = 830 - target_.x_;
            double l3 = target_.y_ + 380;
            double min = l1;
            if (min > l2)
            {
                min = l2;
                if (min > l3)
                {
                    min = l3;
                }
            }
            if (min == l1)
            {
                target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                target_.y_ = 380;
            }
            if (min == l2)
            {
                target_.x_ = 830;
                if (ballPos_.y_ >= 0)
                {
                    target_.y_ = ballPos_.y_ - sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
                else
                {
                    target_.y_ = ballPos_.y_ + sqrt(90000 - (830 - ballPos_.x_) * (830 - ballPos_.x_));
                }
            }
            if (min == l3)
            {
                target_.y_ = -380;
                target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
            }
        }
        if (TeammatesInPenalty() == true)
        {
            if (world_model_->field_info_.isOurPenalty(target_))
            {
                double l1 = 380 - target_.y_;
                double l2 = -830 - target_.x_;
                double l3 = target_.y_ + 380;
                double min = l1;
                if (min > l2)
                {
                    min = l2;
                    if (min > l3)
                    {
                        min = l3;
                    }
                }
                if (min == l1)
                {
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (ballPos_.y_ - 380) * (ballPos_.y_ - 380));
                    target_.y_ = 380;
                }
                if (min == l2)
                {
                    target_.x_ = -830;
                    if (ballPos_.y_ >= 0)
                    {
                        target_.y_ = ballPos_.y_ - sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                    else
                    {
                        target_.y_ = ballPos_.y_ + sqrt(90000 - (-830 - ballPos_.x_) * (-830 - ballPos_.x_));
                    }
                }
                if (min == l3)
                {
                    target_.y_ = -380;
                    target_.x_ = ballPos_.x_ - sqrt(90000 - (-380 - ballPos_.y_) * (-380 - ballPos_.y_));
                }
            }
        }
    }
}

void StaticPass::OurCornerReady_()
{
    int flagy = 1;
    if (ballPos_.y_ < 0.0)
        flagy = -1;
    init_our_in_field();
    target_pos_[0] = DPoint(1100.0, 700.0 * flagy);
    target_pos_[1] = DPoint(900.0, 500.0 * flagy);
    target_pos_[2] = DPoint(500.0, 100.0 * flagy);
    target_pos_[3] = DPoint(400.0, 600.0 * flagy);

    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
    }
}

void StaticPass::OppCornerReady_()
{
    int kickerID = 0;
    int num_in_ok = 0;
    DPoint target__sz[3];
    DPoint ABvector;
    double lengthmin = 10000.0;
    init_our_in_field();
    for (int i = 1; i < 5; i++)
    {
        if ((world_model_->Opponents_[i] - ballPos_).length() < lengthmin && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0))
        {
            kickerID = i;
            lengthmin = (world_model_->Opponents_[i] - ballPos_).length();
        }
    }
    for (int i = 1, j = 0; i < 5; i++)
    {
        if (i != kickerID && world_model_->field_info_.isInInterField2(world_model_->Opponents_[i], 100.0, 100.0) && world_model_->Opponents_[i].x_ < 1000.0)
        {
            ABvector = world_model_->Opponents_[i] - ballPos_;
            if ((ABvector.length() < 700 && (ABvector.length() > 370.0 || ABvector.length() < 250.0)) || world_model_->Opponents_[i].x_ > 0.0)
            {
                target__sz[j] = ballPos_ + (310.0 / ABvector.length()) * ABvector;
            }
            else if (ABvector.length() <= 370.0 && ABvector.length() >= 250.0)
            {
                target__sz[j] = world_model_->Opponents_[i] + (100.0 / ABvector.length()) * ABvector;
            }
            else
            {
                target__sz[j] = world_model_->Opponents_[i] + (-100.0 / ABvector.length()) * ABvector;
            }
            j++;
            num_in_ok++;
        }
    }
    target_pos_[0] = DPoint(-800.0, 300 * world_model_->Opponents_[kickerID].y_ / fabs(world_model_->Opponents_[kickerID].y_));
    target_pos_[1] = DPoint(-1000.0, 400 * world_model_->Opponents_[kickerID].y_ / fabs(world_model_->Opponents_[kickerID].y_));
    target_pos_[2] = DPoint(-300.0, 300 * world_model_->Opponents_[kickerID].y_ / fabs(world_model_->Opponents_[kickerID].y_));
    target_pos_[3] = DPoint(-500.0, 0.0);
    for (int i = 0; i < num_in_ok; i++)
    {
        target_pos_[i] = target__sz[i];
    }
    if (world_model_->AgentID_ == 1)
    {
        target_ = DPoint(-1090.0, 0.0);
    }
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
    }
}

void StaticPass::DropBallReady_() // dropball站位
{
    if (ballPos_.x_ > 200.0 || ballPos_.x_ < -200.0)
    {
        switch (world_model_->AgentID_)
        {
        case 1:
            target_ = DPoint(-1090.0, 0.0);
            break;
        case 2:
            target_ = (ballPos_.x_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 100.0 * (ballPos_.y_) / fabs(ballPos_.y_)) : DPoint(0.0, 0.0);
            break;
        case 3:
            target_ = ballPos_ + DPoint(-50.0 * sqrt(3.0), 50.0 * (ballPos_.y_) / fabs(ballPos_.y_));
            break;
        case 4:
            target_ = ballPos_ + DPoint(50.0 * sqrt(3.0), 50.0 * (ballPos_.y_) / fabs(ballPos_.y_));
            break;
        case 5:
            target_ = (ballPos_.x_ <= 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 100.0 * (ballPos_.y_) / fabs(ballPos_.y_)) : DPoint(0.0, 0.0);
            break;
        }
    }
    else
    {
        switch (world_model_->AgentID_)
        {
        case 1:
            target_ = DPoint(-1090.0, 0.0);
            break;
        case 2:
            target_ = (ballPos_.x_ > 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 100.0 * (ballPos_.y_) / fabs(ballPos_.y_)) : DPoint(200.0, 0.0);
            break;
        case 3:
            target_ = ballPos_ + DPoint(-50.0 * sqrt(3.0), 50.0 * (ballPos_.y_) / fabs(ballPos_.y_));
            break;
        case 4:
            target_ = ballPos_ + DPoint(50.0 * sqrt(3.0), 50.0 * (ballPos_.y_) / fabs(ballPos_.y_));
            break;
        case 5:
            target_ = (ballPos_.x_ <= 0.0) ? DPoint(ballPos_.x_, ballPos_.y_ - 100.0 * (ballPos_.y_) / fabs(ballPos_.y_)) : DPoint(-200.0, 0.0);
            break;
        }
    }
}

void StaticPass::OurDefaultReady_() //我方发球默认的站位
{
}

void StaticPass::OppDefaultReady_() //对方发球默认站位
{
}

void StaticPass::OurPenaltyReady_()
{
}

void StaticPass::OppPenaltyReady_() //对方penalty发球
{
}

void StaticPass::OurkickoffReady_() //我方kickoff发球站位
{
    init_our_in_field();
    target_pos_[0] = DPoint(0.0, 50.0);
    target_pos_[1] = DPoint(0.0, -210.0);
    target_pos_[2] = DPoint(-250.0, 0.0);
    target_pos_[3] = DPoint(-500.0, 0.0);

    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
    }
    // end_our_in_field();
}

void StaticPass::OppkickoffReady_() //对方kickoff发球站位
{
    double OppKickerY;
    double lengthmin = 10000.0;
    for (int i = 1; i < 5; i++)
    {
        if ((world_model_->Opponents_[i] - ballPos_).length() < lengthmin)
        {
            lengthmin = (world_model_->Opponents_[i] - ballPos_).length();
            OppKickerY = world_model_->Opponents_[i].y_;
        }
    }

    init_our_in_field();
    target_pos_[0] = (OppKickerY > 0) ? (DPoint(-300.0, -100.0)) : (DPoint(-300.0, 100.0));
    target_pos_[1] = DPoint(-250.0, -200.0);
    target_pos_[2] = (OppKickerY > 0) ? (DPoint(-500.0, 100.0)) : (DPoint(-500.0, -100.0));
    target_pos_[3] = DPoint(-250.0, 200.0);

    if (world_model_->AgentID_ == 1)
        target_ = DPoint(-1090.0, 0.0);
    else
    {
        for (int i = 0; i < num_in_field; i++)
        {
            if (world_model_->AgentID_ == ID_in_field[i])
            {
                target_ = target_pos_[i];
            }
        }
        for (int j = 0; j < out_of_field; j++)
        {
            if (world_model_->AgentID_ == ID_out_field[j])
            {
                target_ = target_pos_[num_in_field + j];
            }
        }
    }
}

void StaticPass::targetInitialize()
{
    // 给所有站位点赋值，防止后面也没有赋值导致出错
    targetInit_[0] = DPoint(-890, 0); //静态站位时，守门员基本位置恒定
    targetInit_[1] = DPoint(-500, 200);
    targetInit_[2] = DPoint(-500, -200);
    targetInit_[3] = DPoint(-200, 100);
    targetInit_[4] = DPoint(-200, -100);
}

