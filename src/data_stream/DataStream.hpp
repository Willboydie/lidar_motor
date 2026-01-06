#include "Arduino.h"
#include "../motor/Motor.hpp"
#include "../ldm/LDM.hpp"
#include "../utils/tweakable_params.h"

class DataStream {
public:
    DataStream(Motor& _motor1, Motor& _motor2, LDM& _ldm);
    void sendData();

    Motor& motor1;
    Motor& motor2;
    LDM& ldm;

    float calculateAssociatedAngle(Motor& motor);
    
    float associated_angle1 = 0.0f;
    float associated_angle2 = 0.0f;
};