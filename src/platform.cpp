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

void Platform::setVelocity(const vec2f movementSpeed, const double rotationSpeed)
{
    m_movementSpeed = movementSpeed;
    m_rotationSpeed = rotationSpeed;
}

void Platform::setWalkingStyle(StepStyle style)
{
    m_stepStyle = style;
}

Platform::Platform(std::function<void(int)> sleepMsFuction,
                   std::function<void(int, double)> servoPositionFunction,
                   int kinematic_period)
    : m_rotationSpeed(0.0f)
    , m_movementSpeed(0.0f, 0.0f)
    , m_sleepMsFunction(sleepMsFuction)
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

void Platform::movingEnd()
{
    for (size_t i = 0; i < 6; ++i)
    {
        {
            m_legs[i].MoveLegDown();
            m_legs[i].RecalcAngles();
        }
    }
}
/*!
     * \brief Platform::getLegToRaise - find most suitable leg to raise (most far from center)
     * \return leg index or -1
     */
int Platform::getLegToRaise()
{
    int legToRaise = -1;
    double maxDist = 0;
    for (Leg &currentLeg : m_legs)
    {
        double curDist = currentLeg.GetDistanceFromCenter();
        if (curDist > maxDist)
        {
            maxDist = curDist;
            legToRaise = currentLeg.GetLegIndex();
        }
    }
    if (maxDist < minimumDistanceStep)
    {
        legToRaise = -1;
    }
    return legToRaise;
}

void Platform::raiseOneLeg(int legToRaise)
{
    vec2f newPoint(m_legs[legToRaise].GetCenterVec());
    // vec2f tmpOffsetVec=m_movementSpeed * 0.5; //TODO - uncomment for possible optimization
    // newPoint=newPoint+tmpOffsetVec;
    m_legs[legToRaise].MoveLegUp(newPoint);
}

void Platform::raiseTwoLegs(int legToRaise)
{
    raiseOneLeg(legToRaise);
    int secondLegToRaise = (legToRaise < 3) ? (legToRaise+3) : (legToRaise-3);
    raiseOneLeg(secondLegToRaise);
}

void Platform::raiseThreeLegs(int legToRaise)
{
    if (legToRaise%2 == 1)
    {
        raiseOneLeg(1);
        raiseOneLeg(3);
        raiseOneLeg(5);
    }
    else
    {
        raiseOneLeg(0);
        raiseOneLeg(2);
        raiseOneLeg(4);
    }
}

void Platform::procedureGo()
{
    bool anyLegInAir = false;
    for (Leg &currentLeg : m_legs)
    {
        if (currentLeg.leg_position != Leg::on_ground) //for leg in air - move it to center
        {
            anyLegInAir = true;
            currentLeg.ProcessLegMovingInAir();
        }
        else // leg on a ground - move it as needed
        {
            currentLeg.LegAddOffsetInGlobal(m_movementSpeed.x, m_movementSpeed.y);
            currentLeg.TurnLegWithGlobalCoord( m_rotationSpeed );
        }
    }
    if (!anyLegInAir) // all 6 legs on the ground, we check, do we need to raise any leg?
    {

        int legToRaise = getLegToRaise();
        if (legToRaise != -1)
        { // if we have to raise any leg - do it
            switch (m_stepStyle) {
            case OneLeg:
                raiseOneLeg(legToRaise);
                break;
            case TwoLegs:
                raiseTwoLegs(legToRaise);
                break;
            case ThreeLegs:
                raiseThreeLegs(legToRaise);
                break;
            default:
                break;
            }
        }
    }
    for (Leg &currentLeg : m_legs)
    {
        currentLeg.RecalcAngles();
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
        movementDelay();
    }
}

void Platform::movementDelay()
{
    m_sleepMsFunction(m_kinematicPeriod);
}
}
