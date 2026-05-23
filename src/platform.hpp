#pragma once

#include "Leg.hpp"
#include "bodyConfiguration.hpp"
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
                 std::function<void()> readSensorsFunction,
                 int kinematic_period=100);
        /*Move legs into transportable position*/
        void parkLegs();        
        void setVelocity(const vec2f movementSpeed, const double rotationSpeed_deg);
        void setWalkingStyle(StepStyle style);
        void setBodyHeight(const float height);
        float getBodyHeight() const;
        void startMovementThread();
        void stopMovementThread();
        void prepareToGo();
        void setLegCenter(int idx, float x, float y, float height);
        std::pair<float,float> getLegCenter(int idx);
        void setGaitParameters(const bodyConfiguration::GaitParameters& params);
        void procedureGo();
    private:
        void movementThread();
        void movementDelay();
        bool isLegInSwingGroup(int legIndex) const;
    private:
        std::vector<Leg> m_legs;
        double m_bodyHeight;
        vec2f m_targetMovementSpeed;
        vec2f m_currentMovementSpeed;
        double m_targetRotationSpeed_deg;
        double m_currentRotationSpeed_deg;
        double m_gaitPhase_;
        bodyConfiguration::GaitParameters m_gaitParams;
        std::function<void(int)> m_sleepMsFunction;
        std::function<void(int, double)> m_servoPositionFunction;
        std::function<void()> m_readSensorsFunction;
        std::atomic_bool m_active;
        StepStyle m_stepStyle;
        int m_kinematicPeriod;
    };
} //namespace hexaod
