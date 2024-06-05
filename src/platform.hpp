#pragma once

#include "Leg.hpp"
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
        Platform(std::function<void(int)> sleepFuction,
                 std::function<void(int, double)> servoPositionFunction);
        /*Move legs into transportable position*/
        void parkLegs();        
        void setVelocity(const vec2f movementSpeed, const double rotationSpeed);
        void setBodyHeight(const float height);
        float getBodyHeight() const;
        void startMovementThread();
        void stopMovementThread();
        void prepareToGo();
        void setLegCenter(int idx, float x, float y);
        std::pair<float,float> getLegCenter(int idx);
        void procedureGo();
    private:
        void movementThread();
        void movingEnd();
        void movementDelay();
        int getLegToRaise();

    private:

        std::vector<Leg> m_legs;
        double m_bodyHeight;
        double m_rotationSpeed;
        vec2f m_movementSpeed;
        std::function<void(int)> m_sleepMsFunction;
        volatile bool m_active;
    };
} //namespace hexaod

