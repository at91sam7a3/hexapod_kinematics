#pragma once

#include "platform.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace hexapod
{
/*
This is schematic of a robot motors positions

                    FRONT(camera from this side)
 (leg6)     16-15-17    2-0-1 (leg1)
(leg5)   13-12-14         5-3-2 (leg2)
   (leg4)    10-9-11    8-6-7 (leg3)

 Motors position for 1st leg
          1
        /  \
    2-0     \
*/
namespace
{
const double PI = 3.141592654;
const double minimumDistanceStep = 30; // TODO requires experiments
}

// place legs in compact position for transportation
void Platform::parkLegs()
{
    for (unsigned int i = 0; i < 6; ++i)
    {
        m_legs[i].SetMotorAngle(0, 180);
        m_legs[i].SetMotorAngle(1, 0);
        m_legs[i].SetMotorAngle(2, 0);
    }

}

void Platform::setVelocity(const vec2f movementSpeed, const double rotationSpeed_deg)
{
    m_targetMovementSpeed = movementSpeed;
    m_targetRotationSpeed_deg = rotationSpeed_deg;
}

void Platform::setWalkingStyle(StepStyle style)
{
    m_stepStyle = style;
}

void Platform::setGaitParameters(const bodyConfiguration::GaitParameters& params)
{
    m_gaitParams = params;
}

Platform::Platform(std::function<void(int)> sleepMsFuction,
                   std::function<void(int, double)> servoPositionFunction,
                   std::function<void()> readSensorsFunction,
                   int kinematic_period)
    : m_targetRotationSpeed_deg(0.0)
    , m_targetMovementSpeed(0.0, 0.0)
    , m_currentRotationSpeed_deg(0.0)
    , m_currentMovementSpeed(0.0, 0.0)
    , m_gaitPhase_(0.0)
    , m_gaitParams(bodyConfiguration::GaitParameters::getDefault())
    , m_sleepMsFunction(sleepMsFuction)
    , m_servoPositionFunction(servoPositionFunction)
    , m_readSensorsFunction(readSensorsFunction)
    , m_active(false)
    , m_stepStyle(OneLeg)
    , m_kinematicPeriod(kinematic_period)
{
    for (int i = 0; i < 6; ++i)
    {
        Leg leg(servoPositionFunction, i);
        m_legs.push_back(leg);
    }
}

void Platform::setBodyHeight(const float height)
{
    for (size_t i = 0; i < 6; ++i)
    {
        m_legs[i].m_bodyHeight = height;
        m_legs[i].RecalcAngles();
    }
    m_bodyHeight = height;
}

float Platform::getBodyHeight() const
{
    return m_bodyHeight;
}

void Platform::startMovementThread()
{
    if(m_active) return;
    m_active = true;
    std::thread movement(&Platform::movementThread,this);
    movement.detach();
}

void Platform::stopMovementThread()
{
    m_active = false;
}

// Tripod A: indices 0,2,4 (RF, RB, LM) — swings in first half of cycle
// Tripod B: indices 1,3,5 (RM, LB, LF) — swings in second half
bool Platform::isLegInSwingGroup(int legIndex) const
{
    const bool isTripodA = (legIndex % 2 == 0);
    return isTripodA ? (m_gaitPhase_ < 0.5) : (m_gaitPhase_ >= 0.5);
}

void Platform::procedureGo()
{
    // 1. Smooth velocities toward targets
    const double smoothFactor = m_gaitParams.movementSmoothing;
    m_currentMovementSpeed.x += (m_targetMovementSpeed.x - m_currentMovementSpeed.x) * smoothFactor;
    m_currentMovementSpeed.y += (m_targetMovementSpeed.y - m_currentMovementSpeed.y) * smoothFactor;
    m_currentRotationSpeed_deg += (m_targetRotationSpeed_deg - m_currentRotationSpeed_deg) * m_gaitParams.rotationSmoothing;

    // 2. Advance gait phase
    m_gaitPhase_ += m_gaitParams.gaitFrequency;
    if (m_gaitPhase_ >= 1.0)
        m_gaitPhase_ -= 1.0;

    // 3. Process each leg
    for (Leg &leg : m_legs)
    {
        const int idx = leg.GetLegIndex();
        const bool inSwing = isLegInSwingGroup(idx);

        if (inSwing)
        {
            double localPhase;
            const bool isTripodA = (idx % 2 == 0);
            if (isTripodA)
                localPhase = m_gaitPhase_ / 0.5;
            else
                localPhase = (m_gaitPhase_ - 0.5) / 0.5;

            if (!leg.IsSwinging())
            {
                vec2f target = leg.GetCenterVec();
                double speed = m_currentMovementSpeed.size();
                if (speed > 0.1)
                {
                    double stepLen = std::min(speed * 5.0, m_gaitParams.maxStepLength);
                    vec2f stepDir(m_currentMovementSpeed.x / speed, m_currentMovementSpeed.y / speed);
                    target.x += stepDir.x * stepLen;
                    target.y += stepDir.y * stepLen;
                }
                leg.StartSwing(target.x, target.y);
            }
            leg.UpdateSwing(localPhase);
        }
        else
        {
            if (leg.IsSwinging())
                leg.EndSwing();
            leg.LegAddOffsetInGlobal(m_currentMovementSpeed.x, m_currentMovementSpeed.y);
            leg.TurnLegWithGlobalCoord(m_currentRotationSpeed_deg);
        }
    }

    // 4. Recalculate servo angles for all legs
    for (Leg &leg : m_legs)
    {
        leg.RecalcAngles();
    }
}

void Platform::prepareToGo()
{
    for (size_t i = 0; i < 6; ++i)
    {
        if (!m_legs[i].IsInCenter())
        {
            m_legs[i].MoveLegUp();
            movementDelay();
            m_legs[i].MoveLegToCenter();
            movementDelay();
            m_legs[i].RecalcAngles();
            movementDelay();
        }
        m_legs[i].MoveLegDown();
        m_legs[i].RecalcAngles();
        movementDelay();
        movementDelay();
    }
}

void Platform::setLegCenter(int idx, float x, float y, float height =0)
{
    m_legs[idx].SetLocalXY(x,y);
    if(height>0) m_legs[idx].MoveLegUp();
    m_legs[idx].RecalcAngles();

}

std::pair<float, float> Platform::getLegCenter(int idx)
{
    LegCoodinates coord =  m_legs[idx].GetLegCoord();
    return {coord.x, coord.y};
}

void Platform::movementThread()
{
    prepareToGo();
    while (m_active)
    {
        procedureGo();
        if (m_readSensorsFunction)
        {
            m_readSensorsFunction();
        }
        movementDelay();
    }
}

void Platform::movementDelay()
{
    m_sleepMsFunction(m_kinematicPeriod);
}
}
