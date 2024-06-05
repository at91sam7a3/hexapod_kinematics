#include "Leg.hpp"
#include "bodyConfiguration.hpp"
#include <iostream>
#include "vec2f.hpp"

#define DEBUG_LOG

namespace hexapod
{
    Leg::Leg(std::function<void(int, double)> servoFunction, int idx)
        : m_servoFunction(servoFunction),
          m_bodyHeight(50),
          leg_position(on_ground),
          currentLegrotationOffset_(0),
          xPos_(0),
          yPos_(0),
          xCenterPos_(0),
          yCenterPos_(0),
          distanceFromGround_(0),
          m_legIndex(idx),
          movementConfiguration_(bodyConfiguration::HexapodMovementConfiguration::getDefaultSettings()),
          frame_(bodyConfiguration::HexapodFrame::getConfiguredFrame()) 
    {
        //here the motor numbers for this leg
        indexes_.push_back(idx * 3);
        indexes_.push_back(idx * 3 + 1);
        indexes_.push_back(idx * 3 + 2);
        // it means leg look left of right when in math it`s degree is 0 but in real it`s servo 90
        angleCOffsetAccordingToLegAttachment_ = -90;


        //X - front, Y - left(or right)


        // middle legs
        if ((idx == RightMiddle) || (idx == LeftMiddle))
        {
            xCenterPos_ = 0;
            yCenterPos_ = 100;
        }

        if ((idx == RightFront) || (idx == LeftFront))
        {
            xCenterPos_ = 70;
            yCenterPos_ = 70;
        }
        
        if ((idx == RightBack) || (idx == LeftBack))
        {
            xCenterPos_ = -70;//-72-50;
            yCenterPos_ = 70;
        }

        xPos_ = xCenterPos_;
        yPos_ = yCenterPos_;
    }

    void Leg::RecalcAngles()
    {
        #ifdef DEBUG_LOG
        if(m_legIndex == 0)
        {
            std::cout << "FOR LEG #1  x = " << xPos_ << " and y = " << yPos_ << std::endl;
        }

        #endif
        if (yPos_ == 0.0)
            yPos_ = 0.01;
        angleC_ = (atan(xPos_ / yPos_));  //this is angle between body and leg. Servo #2
        double L1 = sqrt(xPos_ * xPos_ + yPos_ * yPos_); //L1 distance from leg attachment to point on ground in 2d
        double L = sqrt(pow(m_bodyHeight, 2.0) + pow((L1 - frame_.cLegPart), 2));
        // angle alpfa
        angleA_ = acos((m_bodyHeight - distanceFromGround_) / L) + acos(((pow(frame_.aLegPart, 2) - pow(frame_.bLegPart, 2) - pow(L, 2))) / (-2 * frame_.bLegPart * L));
        // angle beta
        angleB_ = acos((pow(L, 2) - pow(frame_.aLegPart, 2) - pow(frame_.bLegPart, 2)) / (-2 * frame_.aLegPart * frame_.bLegPart));

        // set angles directly to servos
        angleA_ = angleA_ * 180 / 3.1415;
        angleB_ = angleB_ * 180 / 3.1415;
        angleC_ = angleC_ * 180 / 3.1415;
        SetMotorAngle(0, angleA_);
        SetMotorAngle(1, angleB_);
        SetMotorAngle(2, angleC_);
    }

    void Leg::SetLocalXY(double x, double y) // TODO
    {
        xPos_ = x;
        yPos_ = y;
    }

    void Leg::LegAddOffsetInGlobal(double xoffset, double yoffset)
    {
        xPos_ -= xoffset;
        if (m_legIndex < 3)
        {
            yPos_ += yoffset;
        }
        else
        {
            yPos_ -= yoffset;
        }
    }

    void Leg::SetLegCoord(LegCoodinates &lc)
    {
        SetLocalXY(lc.x, lc.y);
        distanceFromGround_ = lc.height;
    }

    LegCoodinates Leg::GetLegCoord()
    {
        LegCoodinates lc(xPos_, yPos_, distanceFromGround_);
        return lc;
    }

    std::vector<int> Leg::GetMotorIndexes()
    {
        return indexes_;
    }



    double Leg::GetLegDirectionInGlobalCoordinates()
    {

        switch (m_legIndex)
        {
        case RightMiddle:
            return -90;

        case RightFront:
            return -90;

        case RightBack:
            return -90;

        case LeftMiddle:
            return 90;

        case LeftFront:
            return 90;

        case LeftBack:
            return 90;

        default:
            throw std::exception();
            break;
        }
    }

    void Leg::SetMotorAngle(int idx, double angle)
    {
        switch (idx)
        {
        case 0:
            m_servoFunction(indexes_[0], angle);
            break;
        case 1:
            m_servoFunction(indexes_[1], 180 - angle);
            break;
        case 2:
            m_servoFunction(indexes_[2], angle - angleCOffsetAccordingToLegAttachment_);
            break;
        }
    }

    double Leg::GetDistanceFromCenter()
    {
        double xDist = fabs(xPos_ - xCenterPos_);
        double yDist = fabs(yPos_ - yCenterPos_);
        return sqrt(xDist * xDist + yDist * yDist);
    }

    bool Leg::IsInCenter()
    {
        if ((fabs(xPos_ - xCenterPos_) < 0.001) && (fabs(yPos_ - yCenterPos_) < 0.001))
            return true;
        return false;
    }

    void Leg::MoveLegUp()
    {
        distanceFromGround_ = movementConfiguration_.stepHeight;
        leg_position = moving_up;
    }

    void Leg::MoveLegDown()
    {
        distanceFromGround_ = 0;
        leg_position = on_ground;
    }

    void Leg::MoveLegToCenter()
    {
        xPos_ = xCenterPos_;
        yPos_ = yCenterPos_;
    }

    void Leg::MoveLegUp(vec2f newPositionOnGround)
    {
        if (leg_position != on_ground)
            throw(std::runtime_error("try to move up leg that already in air"));
        newPositionOnGround_ = newPositionOnGround;
        distanceFromGround_ = movementConfiguration_.stepHeight;
        leg_position = moving_up;
    }

    void Leg::ProcessLegMovingInAir()
    {
        if (leg_position == moving_up)
        {
            leg_position = moving_to_target;
            SetLocalXY(newPositionOnGround_.x, newPositionOnGround_.y);
            return;
        }
        if (leg_position == moving_to_target)
        {
            leg_position = on_ground;
            distanceFromGround_ = 0;
        }
    }

    int Leg::GetLegIndex()
    {
        return m_legIndex;
    }

    vec2f Leg::GetCenterVec()
    {
        return vec2f(xCenterPos_, yCenterPos_);
    }

    double Leg::GetLegLocalZAngle()
    {
        return angleC_;
    }
    // Next 3 methods are needed for rotation

    void Leg::TurnLegWithGlobalCoord(double offset)
    {
        vec2f currentGlobalPos = GetLegGlobalCoord();
        currentGlobalPos.rotate(offset);
        vec2f lc = GlobalToLocal(currentGlobalPos);
        SetLocalXY(lc.x, lc.y);
    }

    vec2f Leg::GetLegGlobalCoord()
    {
        vec2f res;

        switch (m_legIndex)
        {

        case RightMiddle:
            res.x = xPos_;
            res.y = yPos_ + frame_.centerYOffset;
            break;
        case LeftMiddle:
            res.x = xPos_;
            res.y = -yPos_ - frame_.centerYOffset;
            break;

        case RightFront:
            res.x = xPos_ + frame_.rearXOffset;
            res.y = yPos_ + frame_.rearYOffset;
            break;
        case RightBack:
            res.x = xPos_ - frame_.rearXOffset;
            res.y = yPos_ + frame_.rearYOffset;
            break;

        case LeftFront:
            res.x = xPos_ + frame_.rearXOffset;
            res.y = -yPos_ - frame_.rearYOffset;
            break;
        case LeftBack:
            res.x = xPos_ - frame_.rearXOffset;
            res.y = -yPos_ - frame_.rearYOffset;
            break;

        default:
            break;
        }
        return res;
    }

    // X axis looks front
    // Y axit looks left
    vec2f Leg::GlobalToLocal(vec2f &lc)
    {
        vec2f res;
        switch (m_legIndex)
        {

        case RightMiddle:
            res.x = lc.x;
            res.y = lc.y - frame_.centerYOffset;
            break;
        case LeftMiddle:
            res.x = lc.x;
            res.y = -lc.y - frame_.centerYOffset;
            break;

        case RightFront:
            res.x = lc.x - frame_.rearXOffset;
            res.y = lc.y - frame_.rearYOffset;
            break;
        case RightBack:
            res.x = lc.x + frame_.rearXOffset;
            res.y = lc.y - frame_.rearYOffset;
            break;

        case LeftFront:
            res.x = lc.x - frame_.rearXOffset;
            res.y = -lc.y - frame_.rearYOffset;
            break;
        case LeftBack:
            res.x = lc.x + frame_.rearXOffset;
            res.y = -lc.y - frame_.rearYOffset;
            break;

        default:
            break;
        }
        return res;
    };
}
