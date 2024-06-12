# hexapod_kinematics
 This is a library to calculate hexapod robot movement

 Here the pictures provides schematic body sizes and servo numbers

![plot](./doc/body.png)

![plot](./doc/motor-orders.png)

 You need to create instance of Platorm clss and pass there 2 functors - one for setting angle to joint and one for syystm specific sleep in ms.

Here is a simplest code to make robot move forward:
```
#include "../../hexapod_kinematics/src/platform.hpp"

void setServo(int id, double angle)
{
    if(id>8) angle = 180 - angle;
    yourFunctionToCOntrolServoAngle(idx,angle);
}

void sleepMs(int sleepTime)
{
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
}

int main(int argc, char *argv[])
{
    hexapod::Platform platform(&sleepMs, &setServo);
    platform.setVelocity({1, 0}, 0); //front,size,rotation, {0,0},1 means rotate on place
    platform.startMovementThread();
    ...wait here...
}
```