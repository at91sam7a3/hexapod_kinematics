#pragma once
#include <vector>
#include <functional>
#include "vec2f.hpp"
#include "bodyConfiguration.hpp"


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

        void SetLegCoord(const LegCoodinates& lc);

        bool IsInCenter() const;

        void MoveLegUp();

        void MoveLegDown();

        void MoveLegToCenter();

        void MoveLegUp(vec2f newPositionOnGround);

        void SetMotorAngle(int idx, double angle_deg);

        void ProcessLegMovingInAir();

        void StartSwing(double targetX, double targetY);

        void UpdateSwing(double phase);

        void EndSwing();

        bool IsSwinging() const { return swingPhase_ > 0.0; }

        double GetSwingPhase() const { return swingPhase_; }

        int GetLegIndex() const;

        vec2f GetCenterVec() const;

        double GetDistanceFromCenter() const;

        void TurnLegWithGlobalCoord(double offset_deg);

        double m_bodyHeight;

        LegCoodinates GetLegCoord() const;

    private:
        enum LegPosition
        {
            on_ground = 0,
            moving_up,
            moving_to_target,
            moving_down
        } leg_position;

        vec2f GlobalToLocal(const vec2f& lc) const;

        double GetLegDirectionInGlobalCoordinates() const;

        float currentLegrotationOffset_deg;

        double GetLegLocalZAngle() const;

        vec2f GetLegGlobalCoord() const;

    private:
        std::function<void(int, double)> m_servoFunction;
        volatile double xPos_;
        volatile double yPos_;
        double xCenterPos_;
        double yCenterPos_;
        vec2f newPositionOnGround_;
        double distanceFromGround_;
        // output, angles in degrees
        double angleA_deg;
        double angleB_deg;
        double angleC_deg;
        // setted servos numbers
        std::vector<int> indexes_;
        int m_legIndex;
        float angleCOffsetAccordingToLegAttachment_deg;
        // swing state
        double swingPhase_;
        double swingStartX_;
        double swingStartY_;
        double swingTargetX_;
        double swingTargetY_;
        bodyConfiguration::HexapodMovementConfiguration movementConfiguration_;
        bodyConfiguration::HexapodFrame frame_;
    };
}
