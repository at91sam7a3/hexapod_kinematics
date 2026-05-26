#pragma once

#include "Leg.hpp"
#include "bodyConfiguration.hpp"
#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <mutex>

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

        Platform(std::function<void(int)> sleepMsFunction,
                 std::function<void(int, double)> servoPositionFunction,
                 std::function<void()> readSensorsFunction,
                 int kinematic_period=100);

        ~Platform();

        /*Move legs into transportable position*/
        void parkLegs();
        void setVelocity(const vec2f& movementSpeed, double rotationSpeed_deg);
        void setWalkingStyle(StepStyle style);
        void setBodyHeight(const float height);
        float getBodyHeight() const;
        void startMovementThread();
        void stopMovementThread();
        void prepareToGo();
        void setLegCenter(int idx, float x, float y, float height);
        std::pair<float,float> getLegCenter(int idx);
        void setGaitParameters(const bodyConfiguration::GaitParameters& params);
        void setTrajectoryType(TrajectoryType type);
        TrajectoryType getTrajectoryType() const { return m_trajectoryType; }
        void setBodyPitch(double deg);
        void setBodyRoll(double deg);
        double getBodyPitch() const;
        double getBodyRoll() const;
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
        double m_targetBodyPitch_deg;
        double m_targetBodyRoll_deg;
        double m_currentBodyPitch_deg;
        double m_currentBodyRoll_deg;
        double m_gaitPhase_;
        bodyConfiguration::GaitParameters m_gaitParams;
        std::function<void(int)> m_sleepMsFunction;
        std::function<void(int, double)> m_servoPositionFunction;
        std::function<void()> m_readSensorsFunction;
        std::atomic_bool m_active;
        StepStyle m_stepStyle;
        TrajectoryType m_trajectoryType = TrajectoryType::LinearSine;
        int m_kinematicPeriod;
        std::thread m_movementThread;
        mutable std::mutex m_mutex;
    };
} //namespace hexaod
