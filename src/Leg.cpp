#include "Leg.hpp"
#include "bodyConfiguration.hpp"
#include <cmath>
#include "vec2f.hpp"
#include <stdexcept>
#include <iostream>

namespace
{
    constexpr double PI = 3.141592654;
}

namespace hexapod
{
Leg::Leg(std::function<void(int, double)> servoFunction, int idx)
    : m_servoFunction(servoFunction),
    m_bodyHeight(50),
    leg_position(on_ground),
    currentLegrotationOffset_deg(0),
    xPos_(0),
    yPos_(0),
    xCenterPos_(0),
    yCenterPos_(0),
    distanceFromGround_(0),
    swingPhase_(0),
    swingStartX_(0),
    swingStartY_(0),
    swingTargetX_(0),
    swingTargetY_(0),
    m_legIndex(idx),
    movementConfiguration_(bodyConfiguration::HexapodMovementConfiguration::getDefaultSettings()),
    frame_(bodyConfiguration::HexapodFrame::getConfiguredFrame())
{
    //here the motor numbers for this leg
    indexes_.push_back(idx * 3);
    indexes_.push_back(idx * 3 + 1);
    indexes_.push_back(idx * 3 + 2);
    // it means leg look left of right when in math it`s degree is 0 but in real it`s servo 90
    angleCOffsetAccordingToLegAttachment_deg = -90;
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
    double angleC_rad = std::atan2(xPos_, yPos_);
    double L1 = std::sqrt(xPos_ * xPos_ + yPos_ * yPos_);
    double dh = m_bodyHeight - distanceFromGround_;
    double dL = L1 - frame_.cLegPart;
    double L = std::sqrt(dh * dh + dL * dL);
    if (L > (frame_.aLegPart + frame_.bLegPart))
        return;

    double aSq = frame_.aLegPart * frame_.aLegPart;
    double bSq = frame_.bLegPart * frame_.bLegPart;
    double angleA_rad = std::acos(dh / L) + std::acos((aSq - bSq - L * L) / (-2.0 * frame_.bLegPart * L));
    double angleB_rad = std::acos((L * L - aSq - bSq) / (-2.0 * frame_.aLegPart * frame_.bLegPart));

    constexpr double radToDeg = 180.0 / PI;
    angleA_deg = angleA_rad * radToDeg;
    angleB_deg = angleB_rad * radToDeg;
    angleC_deg = angleC_rad * radToDeg;
    SetMotorAngle(0, angleA_deg);
    SetMotorAngle(1, angleB_deg);
    SetMotorAngle(2, angleC_deg);
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

void Leg::SetLegCoord(const LegCoodinates& lc)
{
    SetLocalXY(lc.x, lc.y);
    distanceFromGround_ = lc.height;
}

LegCoodinates Leg::GetLegCoord() const
{
    LegCoodinates lc(xPos_, yPos_, distanceFromGround_);
    return lc;
}

double Leg::GetLegDirectionInGlobalCoordinates() const
{

    switch (m_legIndex)
    {
    case RightMiddle:
    case RightFront:
    case RightBack:
        return -90;
    case LeftMiddle:
    case LeftFront:
    case LeftBack:
        return 90;

    default:
        throw std::runtime_error("wrong leg number");
        break;
    }
}

void Leg::SetMotorAngle(int idx, double angle_deg)
{
    try {
        double finalAngle_deg = 0;
        switch (idx)
        {
        case 0:
            finalAngle_deg = angle_deg;
            break;
        case 1:
            finalAngle_deg = 180 - angle_deg;
            break;
        case 2:
            finalAngle_deg = angle_deg - angleCOffsetAccordingToLegAttachment_deg;
            break;
        default:
            throw(std::runtime_error("Wrong motor index"));
        }

        if (finalAngle_deg < 0) finalAngle_deg = 0;
        if (finalAngle_deg > 180) finalAngle_deg = 180;
        m_servoFunction(indexes_[idx], finalAngle_deg);
    }
    catch(std::runtime_error& e)
    {
        std::cerr<<e.what()<<std::endl;
        std::cerr<<"error situation: "<<std::endl;
        std::cerr<<" xPos = "<<xPos_<<" yPos = "<<yPos_<<std::endl;
        std::cerr<<"angle = "<<angle_deg<<" idx = "<<idx<<std::endl;
        std::cerr<<"angleA = "<<angleA_deg<<std::endl;
        std::cerr<<"angleB = "<<angleB_deg<<std::endl;
        std::cerr<<"angleC = "<<angleC_deg<<std::endl;
        std::cerr<<"m_bodyHeight = "<<m_bodyHeight<<" distanceFromGround_ = "<<distanceFromGround_<<std::endl;
    }
}

double Leg::GetDistanceFromCenter() const
{
    double xDist = std::abs(xPos_ - xCenterPos_);
    double yDist = std::abs(yPos_ - yCenterPos_);
    return std::sqrt(xDist * xDist + yDist * yDist);
}

bool Leg::IsInCenter() const
{
    return (std::abs(xPos_ - xCenterPos_) < 0.001) && (std::abs(yPos_ - yCenterPos_) < 0.001);
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

void Leg::StartSwing(double targetX, double targetY)
{
    swingStartX_ = xPos_;
    swingStartY_ = yPos_;
    swingTargetX_ = targetX;
    swingTargetY_ = targetY;
    swingPhase_ = 0.001;
    distanceFromGround_ = movementConfiguration_.stepHeight * sin(swingPhase_ * PI);
    leg_position = moving_up;
}

void Leg::UpdateSwing(double phase)
{
    if (phase >= 1.0)
    {
        EndSwing();
        return;
    }
    swingPhase_ = phase;
    double zFactor = sin(swingPhase_ * PI);
    distanceFromGround_ = movementConfiguration_.stepHeight * zFactor;
    xPos_ = swingStartX_ + (swingTargetX_ - swingStartX_) * swingPhase_;
    yPos_ = swingStartY_ + (swingTargetY_ - swingStartY_) * swingPhase_;
}

void Leg::EndSwing()
{
    swingPhase_ = 0.0;
    distanceFromGround_ = 0.0;
    xPos_ = swingTargetX_;
    yPos_ = swingTargetY_;
    leg_position = on_ground;
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

int Leg::GetLegIndex() const
{
    return m_legIndex;
}

vec2f Leg::GetCenterVec() const
{
    return vec2f(xCenterPos_, yCenterPos_);
}

double Leg::GetLegLocalZAngle() const
{
    return angleC_deg;
}
// Next 3 methods are needed for rotation

void Leg::TurnLegWithGlobalCoord(double offset_deg)
{
    vec2f currentGlobalPos = GetLegGlobalCoord();
    currentGlobalPos.rotate(offset_deg);
    vec2f lc = GlobalToLocal(currentGlobalPos);
    SetLocalXY(lc.x, lc.y);
}

vec2f Leg::GetLegGlobalCoord() const
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
vec2f Leg::GlobalToLocal(const vec2f& lc) const
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
}
}
