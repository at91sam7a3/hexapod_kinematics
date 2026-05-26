#include "platform.hpp"
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
constexpr double minimumDistanceStep = 30;
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

void Platform::setVelocity(const vec2f& movementSpeed, double rotationSpeed_deg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_targetMovementSpeed = movementSpeed;
    m_targetRotationSpeed_deg = rotationSpeed_deg;
}

void Platform::setWalkingStyle(StepStyle style)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stepStyle = style;
}

void Platform::setGaitParameters(const bodyConfiguration::GaitParameters& params)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gaitParams = params;
}

void Platform::setTrajectoryType(TrajectoryType type)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_trajectoryType = type;
    for (Leg& leg : m_legs)
    {
        leg.setTrajectoryType(type);
    }
}

Platform::Platform(std::function<void(int)> sleepMsFunction,
                   std::function<void(int, double)> servoPositionFunction,
                   std::function<void()> readSensorsFunction,
                   int kinematic_period)
    : m_targetRotationSpeed_deg(0.0)
    , m_targetMovementSpeed(0.0, 0.0)
    , m_currentRotationSpeed_deg(0.0)
    , m_currentMovementSpeed(0.0, 0.0)
    , m_gaitPhase_(0.0)
    , m_gaitParams(bodyConfiguration::GaitParameters::getDefault())
    , m_sleepMsFunction(sleepMsFunction)
    , m_servoPositionFunction(servoPositionFunction)
    , m_readSensorsFunction(readSensorsFunction)
    , m_active(false)
    , m_stepStyle(ThreeLegs)
    , m_kinematicPeriod(kinematic_period)
{
    for (int i = 0; i < 6; ++i)
    {
        Leg leg(servoPositionFunction, i);
        m_legs.push_back(leg);
    }
}

void Platform::setBodyHeight(float height)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    for (Leg& leg : m_legs)
    {
        leg.m_bodyHeight = height;
        leg.RecalcAngles();
    }
    m_bodyHeight = height;
}

float Platform::getBodyHeight() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_bodyHeight;
}

Platform::~Platform()
{
    stopMovementThread();
}

void Platform::startMovementThread()
{
    if(m_active) return;
    m_active = true;
    if (m_movementThread.joinable())
    {
        m_movementThread.join();
    }
    m_movementThread = std::thread(&Platform::movementThread, this);
}

void Platform::stopMovementThread()
{
    m_active = false;
    if (m_movementThread.joinable())
    {
        m_movementThread.join();
    }
}

// Tripod A: indices 0,2,4 (RF, RB, LM) — swings in first half of cycle
// Tripod B: indices 1,3,5 (RM, LB, LF) — swings in second half
bool Platform::isLegInSwingGroup(int legIndex) const
{
    const bool isTripodA = (legIndex % 2 == 0);
    return isTripodA ? (m_gaitPhase_ < m_gaitParams.swingRatio) : (m_gaitPhase_ >= m_gaitParams.swingRatio);
}

void Platform::procedureGo()
{
    constexpr double motionThreshold = 0.01;
    constexpr double speedThreshold = 0.05;

    vec2f targetMovementSpeed;
    double targetRotationSpeed_deg;
    bodyConfiguration::GaitParameters gp;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        targetMovementSpeed = m_targetMovementSpeed;
        targetRotationSpeed_deg = m_targetRotationSpeed_deg;
        gp = m_gaitParams;
    }

    // 1. Smooth velocities toward targets
    const double smoothFactor = gp.movementSmoothing;
    m_currentMovementSpeed.x += (targetMovementSpeed.x - m_currentMovementSpeed.x) * smoothFactor;
    m_currentMovementSpeed.y += (targetMovementSpeed.y - m_currentMovementSpeed.y) * smoothFactor;
    m_currentRotationSpeed_deg += (targetRotationSpeed_deg - m_currentRotationSpeed_deg) * gp.rotationSmoothing;

    const double swingRatio = gp.swingRatio;

    // Calculate expected frames per step for proper distance matching
    // gaitFrequency is phase increment per frame; 1.0 = full cycle
    double framesPerFullCycle = 1.0 / gp.gaitFrequency;
    double framesPerStance = framesPerFullCycle * (1.0 - swingRatio);

    // 2. Only advance gait if motion is meaningful or legs have drifted from center
    double motionMag = std::abs(m_currentMovementSpeed.x) + std::abs(m_currentMovementSpeed.y)
                     + std::abs(m_currentRotationSpeed_deg) * 2.0;
    // Also check target speed separately - if user is requesting motion, start gait
    double targetMotionMag = std::abs(targetMovementSpeed.x) + std::abs(targetMovementSpeed.y)
                            + std::abs(targetRotationSpeed_deg) * 2.0;
    bool needsStep = (motionMag > motionThreshold) || (targetMotionMag > motionThreshold);
    if (!needsStep)
    {
        for (Leg &leg : m_legs)
        {
            if (leg.GetDistanceFromCenter() > minimumDistanceStep)
            {
                needsStep = true;
                break;
            }
        }
    }

    if (needsStep)
    {
        m_gaitPhase_ += gp.gaitFrequency;
        if (m_gaitPhase_ >= 1.0)
            m_gaitPhase_ -= 1.0;
    }

    // 3. Process each leg + recalc angles in a single pass
    const vec2f& curSpeed = m_currentMovementSpeed;
    const double curRotSpeed = m_currentRotationSpeed_deg;
    for (Leg &leg : m_legs)
    {
        const int idx = leg.GetLegIndex();
        const bool inSwing = isLegInSwingGroup(idx);

        if (inSwing)
        {
            if (!leg.IsSwinging())
            {
                if (!needsStep)
                {
                    leg.RecalcAngles();
                    continue;
                }
                vec2f target = leg.GetCenterVec();
                double speed = curSpeed.size();
                if (speed > speedThreshold)
                {
                    // Step length should match how far the stance leg moves:
                    // speed * framesPerStance, but we also need to account for
                    // the leg landing slightly ahead of center for smooth motion
                    double stepLen = std::min(speed * framesPerStance, gp.maxStepLength);
                    vec2f stepDir(curSpeed.x / speed, curSpeed.y / speed);
                    target.x += stepDir.x * stepLen;
                    target.y += stepDir.y * stepLen;
                }
                leg.StartSwing(target.x, target.y);
            }

            if (!needsStep)
            {
                double p = leg.GetSwingPhase() + gp.gaitFrequency / swingRatio;
                if (p >= 1.0)
                    leg.EndSwing();
                else
                    leg.UpdateSwing(p);
            }
            else
            {
                double localPhase;
                const bool isTripodA = (idx % 2 == 0);
                if (isTripodA)
                    localPhase = m_gaitPhase_ / swingRatio;
                else
                    localPhase = (m_gaitPhase_ - swingRatio) / (1.0 - swingRatio);
                leg.UpdateSwing(localPhase);
            }
        }
        else
        {
            if (leg.IsSwinging())
                leg.EndSwing();
            leg.LegAddOffsetInGlobal(curSpeed.x, curSpeed.y);
            leg.TurnLegWithGlobalCoord(curRotSpeed);
        }
        leg.RecalcAngles();
    }
}

void Platform::prepareToGo()
{
    for (int i = 0; i < 6; ++i)
    {
        Leg& leg = m_legs[i];
        if (!leg.IsInCenter())
        {
            leg.MoveLegUp();
            movementDelay();
            leg.MoveLegToCenter();
            movementDelay();
            leg.RecalcAngles();
            movementDelay();
        }
        leg.MoveLegDown();
        leg.RecalcAngles();
        movementDelay();
        movementDelay();

        if (i < 5)
            m_sleepMsFunction(500);
    }
}

void Platform::setLegCenter(int idx, float x, float y, float height)
{
    m_legs[idx].SetLocalXY(x, y);
    if (height > 0)
        m_legs[idx].MoveLegUp();
    m_legs[idx].RecalcAngles();
}

std::pair<float, float> Platform::getLegCenter(int idx)
{
    LegCoodinates coord = m_legs[idx].GetLegCoord();
    return {static_cast<float>(coord.x), static_cast<float>(coord.y)};
}

void Platform::movementThread()
{
    prepareToGo();
    while (m_active)
    {
        procedureGo();
        if (m_readSensorsFunction)
        {
            m_readSensorsFunction();
        }
        movementDelay();
    }
}

void Platform::movementDelay()
{
    m_sleepMsFunction(m_kinematicPeriod);
}
}
