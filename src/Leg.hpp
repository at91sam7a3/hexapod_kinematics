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
        /*!
         * \brief RecalcAngles update new servo angles depending on a end of a leg position.
         *        Needed to be called after and leg coordinates changes
         */
        void RecalcAngles();
        /*!
         * \brief SetLocalXY this method control leg end position
         */
        void SetLocalXY(double, double);
        /*!
         * \brief LegAddOffsetInGlobal - with this method we move end of our leg in needed direction
         */
        void LegAddOffsetInGlobal(double, double);
        /*!
         * \brief SetLegCoord
         * \param lc - setter for leg coordinates
         */
        void SetLegCoord(LegCoodinates &lc);
        /*!
         * \brief IsInCenter - this method checks - is leg coordinates already in the senter zone
         * \return
         */
        bool IsInCenter();
        /*!
         * \brief MoveLegUp - Raise the leg
         */
        void MoveLegUp();
        /*!
         * \brief MoveLegDown - move leg to the ground level - 0 height
         */
        void MoveLegDown();
        /*!
         * \brief MoveLegToCenter - move leg position to the center
         */
        void MoveLegToCenter();        
        void MoveLegUp(vec2f newPositionOnGround);
        void SetMotorAngle(int idx, double angle_deg);
        void ProcessLegMovingInAir();
        void StartSwing(double targetX, double targetY);
        void UpdateSwing(double phase);
        void EndSwing();
        bool IsSwinging() const { return swingPhase_ > 0.0; }
        int GetLegIndex();
        vec2f GetCenterVec();
        double GetDistanceFromCenter();
        void TurnLegWithGlobalCoord(double offset_deg);
        enum LegPosition
        {
            on_ground = 0,
            moving_up,
            moving_to_target,
            moving_down
        } leg_position;
        double m_bodyHeight;
        LegCoodinates GetLegCoord();
    private:
        // convert global coordinates to local for this leg
        vec2f GlobalToLocal(vec2f &lc);
        // get Leg angle
        double GetLegDirectionInGlobalCoordinates();
        // this is needed only for rotating procesure
        float currentLegrotationOffset_deg;
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
