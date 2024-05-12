#pragma once

#include "Leg.hpp"
#include <memory>
#include <thread>
#include <vector>
#include <functional>

// This class manage all movements of robot
// It contain it`s own thread, and really autonomous
namespace hexapod
{
    class Platform
    {
    public:
        Platform(std::function<void(int)> sleepFuction,
                 std::function<void(int, double)> servoPositionFunction);
        void ParkLegs();
        void setVelocity(const vec2f movementSpeed, const double rotationSpeed);
        void SetBodyHeight(const float height);
        float GetBodyHeight() const;
        void MovementThread();

    private:
        void prepareToGo();
        void movingEnd();
        void procedureGo();
        void MovementDelay();
        int getLegToRaise();

    private:
        std::unique_ptr<std::thread> moving_thread_;
        std::vector<Leg> m_legs;
        double bodyHeight_;
        double m_rotationSpeed;
        vec2f m_movementSpeed;
        std::function<void(int)> m_sleepMsFunction;
    };
} //namespace hexaod

