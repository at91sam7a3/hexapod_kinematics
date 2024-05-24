#include "platform.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace hexapod
{

    static const double minimumDistanceStep = 30; // TODO requires experiments

    const double PI = 3.141592654;

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

    Platform::Platform(std::function<void(int)> sleepMsFuction,
                       std::function<void(int, double)> servoPositionFunction)
        : m_rotationSpeed(0.0f), m_movementSpeed(0.0f, 0.0f), m_sleepMsFunction(sleepMsFuction), m_active(false)
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
        std::thread movement(&Platform::movementThread);
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
        if (maxDist > minimumDistanceStep)
        {
            legToRaise = -1;
        }
        return legToRaise;
    }

    void Platform::procedureGo()
    {
        bool anyLegInAir = false;
        for (Leg &currentLeg : m_legs)
        {
            if (currentLeg.leg_position != Leg::on_ground)
            {
                anyLegInAir = true;
                currentLeg.ProcessLegMovingInAir();
            }
            else // leg on a ground
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
                vec2f newPoint(m_legs[legToRaise].GetCenterVec());
                // vec2f tmpOffsetVec=m_movementSpeed * 0.5; //TODO - uncomment for possible optimization
                // newPoint=newPoint+tmpOffsetVec;
                m_legs[legToRaise].MoveLegUp(newPoint);
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
                m_legs[i].MoveLegToCenter();
                m_legs[i].RecalcAngles();
                movementDelay();
            }
            m_legs[i].MoveLegDown();
            m_legs[i].RecalcAngles();
            movementDelay();
            movementDelay();
        }
    }

    void Platform::movementThread()
    {        
        prepareToGo();
        while (1)
        {
            procedureGo();
        }
    }

    void Platform::movementDelay()
    {
        m_sleepMsFunction(100);
    }
}
