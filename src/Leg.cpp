#include "Leg.hpp"
#include "bodyConfiguration.hpp"
#include <cmath>
#include "vec2f.hpp"
#include <stdexcept>
#include <iostream>

namespace
{
    constexpr double degToRad = bodyConfiguration::PI / 180.0;
    constexpr double radToDeg = 180.0 / bodyConfiguration::PI;
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

    xCenterPos_ = frame_.legCenterX[idx];
    yCenterPos_ = frame_.legCenterY[idx];

    xPos_ = xCenterPos_;
    yPos_ = yCenterPos_;
}

IKResult Leg::RecalcAngles()
{
    double xEff = xPos_;
    double yEff = yPos_;
    double dhEff = m_bodyHeight - distanceFromGround_;

    double pitchRad = m_bodyPitch_deg * degToRad;
    double rollRad = m_bodyRoll_deg * degToRad;
    if (std::abs(pitchRad) > 1e-9 || std::abs(rollRad) > 1e-9)
    {
        double cosP = std::cos(pitchRad);
        double sinP = std::sin(pitchRad);
        double cosR = std::cos(rollRad);
        double sinR = std::sin(rollRad);

        double x = xPos_;
        double y = yPos_;
        double z = distanceFromGround_ - m_bodyHeight;
        double ySigned = (m_legIndex < 3) ? y : -y;

        // Rotate foot-to-shoulder vector from body frame to world frame
        // R = Ry(-pitch) * Rx(-roll) transforms body->world
        double xWorld = cosP * x + sinP * sinR * ySigned - sinP * cosR * z;
        double yWorld = cosR * ySigned + sinR * z;
        double zWorld = sinP * x - cosP * sinR * ySigned + cosP * cosR * z;

         xEff = xWorld;
         yEff = (m_legIndex < 3) ? yWorld : -yWorld;
         dhEff = -zWorld;
    }

    double angleC_rad = std::atan2(xEff, yEff);
    double L1 = std::sqrt(xEff * xEff + yEff * yEff);
    double dL = L1 - frame_.cLegPart;
    double L = std::sqrt(dhEff * dhEff + dL * dL);

    const double maxReach = frame_.aLegPart + frame_.bLegPart;
    const double minReach = std::abs(frame_.aLegPart - frame_.bLegPart);

    if (L > maxReach + 1e-6)
    {
        lastIKResult_ = IKResult::OutOfReach;
        return IKResult::OutOfReach;
    }

    if (L < minReach - 1e-6)
    {
        lastIKResult_ = IKResult::TooClose;
        return IKResult::TooClose;
    }

    if (L < 1e-6 || std::abs(L - maxReach) < 1e-6 || std::abs(L - minReach) < 1e-6)
    {
        lastIKResult_ = IKResult::Singularity;
    }

    double aSq = frame_.aLegPart * frame_.aLegPart;
    double bSq = frame_.bLegPart * frame_.bLegPart;
    double angleA_rad = std::acos(dhEff / L) + std::acos((aSq - bSq - L * L) / (-2.0 * frame_.bLegPart * L));
    double angleB_rad = std::acos((L * L - aSq - bSq) / (-2.0 * frame_.aLegPart * frame_.bLegPart));

    angleA_deg = angleA_rad * radToDeg;
    angleB_deg = angleB_rad * radToDeg;
    angleC_deg = angleC_rad * radToDeg;

    bool clamped = false;
    if (angleA_deg < 0 || angleA_deg > 180) clamped = true;
    if (angleB_deg < 0 || angleB_deg > 180) clamped = true;
    if (angleC_deg < 0 || angleC_deg > 180) clamped = true;

    SetMotorAngle(0, angleA_deg);
    SetMotorAngle(1, angleB_deg);
    SetMotorAngle(2, angleC_deg);

    if (clamped)
    {
        lastIKResult_ = IKResult::Clamped;
        return IKResult::Clamped;
    }

    lastIKResult_ = IKResult::Success;
    return IKResult::Success;
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
    distanceFromGround_ = movementConfiguration_.stepHeight * sin(swingPhase_ * bodyConfiguration::PI);
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

    double xyFactor;
    double zFactor;

    switch (m_trajectoryType)
    {
        case TrajectoryType::Cycloid: {
            const double theta = phase * 2.0 * bodyConfiguration::PI;
            xyFactor = (theta - std::sin(theta)) / (2.0 * bodyConfiguration::PI);
            zFactor = 0.5 * (1.0 - std::cos(phase * bodyConfiguration::PI));
            break;
        }
        case TrajectoryType::LinearSine:
        default:
            xyFactor = phase;
            zFactor = std::sin(phase * bodyConfiguration::PI);
            break;
    }

    distanceFromGround_ = movementConfiguration_.stepHeight * zFactor;
    xPos_ = swingStartX_ + (swingTargetX_ - swingStartX_) * xyFactor;
    yPos_ = swingStartY_ + (swingTargetY_ - swingStartY_) * xyFactor;
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

 double Leg::getMaxReach() const
 {
     return frame_.aLegPart + frame_.bLegPart;
 }

 double Leg::getMinReach() const
 {
     return std::abs(frame_.aLegPart - frame_.bLegPart);
 }

 bool Leg::isReachable(double x, double y, double height) const
 {
     double L1 = std::sqrt(x * x + y * y);
     double dh = m_bodyHeight - height;
     double dL = L1 - frame_.cLegPart;
     double L = std::sqrt(dh * dh + dL * dL);

     const double maxReach = frame_.aLegPart + frame_.bLegPart;
     const double minReach = std::abs(frame_.aLegPart - frame_.bLegPart);

     return (L <= maxReach + 1e-6) && (L >= minReach - 1e-6);
 }
}
