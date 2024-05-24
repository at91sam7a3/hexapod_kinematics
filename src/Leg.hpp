#pragma once
#include <math.h>
#include <vector>
#include <iostream>
#include <functional>
#include "vec2f.hpp"

namespace hexapod
{

    enum Legs
    {
        RightFront = 0,
        RightMiddle,
        RightBack,
        LeftBack,
        LeftMiddle,
        LeftFront
    };

    struct LegCoodinates
    {
        LegCoodinates(double x1, double y1, double h1)
            : x(x1), y(y1), height(h1)
        {
        }
        LegCoodinates()
            : x(0), y(0), height(0)
        {
        }
        double x;
        double y;
        double height;
    };

    class Leg
    {
    public:
        Leg(std::function<void(int, double)> servoFunction, int legIndex);
        void RecalcAngles();
        void SetLocalXY(double, double);
        void LegAddOffsetInGlobal(double, double);
        void SetLegCoord(LegCoodinates &lc);
        bool IsInCenter();
        void MoveLegUp();
        void MoveLegDown();
        void MoveLegToCenter();
        void MoveLegUp(vec2f newPositionOnGround);
        void SetMotorAngle(int idx, double angle);
        void ProcessLegMovingInAir();
        int GetLegIndex();
        vec2f GetCenterVec();
        double GetDistanceFromCenter();
        void TurnLegWithGlobalCoord(double offset);
        enum LegPosition
        {
            on_ground = 0,
            moving_up,
            moving_to_target,
            moving_down
        } leg_position;
        double m_bodyHeight;
    private:
        LegCoodinates GetLegCoord();
        std::vector<int> GetMotorIndexes();
        // convert global coordinates to local for this leg
        vec2f GlobalToLocal(vec2f &lc);
        // get Leg angle
        double GetLegDirectionInGlobalCoordinates();
        // this is needed only for rotating procesure
        float currentLegrotationOffset_;
        double GetLegLocalZAngle();
        vec2f GetLegGlobalCoord();

    private:
        std::function<void(int, double)> m_servoFunction;
        volatile double xPos_;
        volatile double yPos_;
        double xCenterPos_;
        double yCenterPos_;
        vec2f newPositionOnGround_;
        double distanceFromGround_;
        // output, angles in radians
        double angleA_;
        double angleB_;
        double angleC_;
        // setted servos numbers
        std::vector<int> indexes_;
        int m_legIndex;
        float angleCOffsetAccordingToLegAttachment_;
    };
}
