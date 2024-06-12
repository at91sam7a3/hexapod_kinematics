#pragma once

#include "Leg.hpp"
#include <atomic>
#include <memory>
#include <thread>
#include <vector>
#include <functional>

// This class manage all movements of robot
namespace hexapod
{
    class Platform
    {
    public:
        enum StepStyle
        {
            OneLeg,
            TwoLegs,
            ThreeLegs
        };

        Platform(std::function<void(int)> sleepFuction,
                 std::function<void(int, double)> servoPositionFunction,
                 int kinematic_period=100);
        /*Move legs into transportable position*/
        void parkLegs();        
        void setVelocity(const vec2f movementSpeed, const double rotationSpeed);
        void setWalkingStyle(StepStyle style);
        void setBodyHeight(const float height);
        float getBodyHeight() const;
        void startMovementThread();
        void stopMovementThread();
        void prepareToGo();
        void setLegCenter(int idx, float x, float y, float height);
        std::pair<float,float> getLegCenter(int idx);
        void procedureGo();
    private:
        void movementThread();
        void movingEnd();
        void movementDelay();
        int getLegToRaise();
        void raiseOneLeg(int legToRaise);
        void raiseTwoLegs(int legToRaise);
        void raiseThreeLegs(int legToRaise);
    private:
        std::vector<Leg> m_legs;
        double m_bodyHeight;
        double m_rotationSpeed;
        vec2f m_movementSpeed;
        std::function<void(int)> m_sleepMsFunction;
        std::atomic_bool m_active;
        StepStyle m_stepStyle;
        int m_kinematicPeriod;
    };
} //namespace hexaod

