#pragma once

#include "state_machine/StateMachine.hpp"
#include "foc_controller/FOCController.hpp"
#include "motor/Motor.hpp"
#include "ldm/LDM.hpp"
#include "sensor/Sensor.hpp"
#include "scan_path/ScanPath.hpp"
#include "current_sense/InlineCurrentSense.hpp"

class System {

public:
    System(StateMachine& stateMachine, 
        FOCController& focController1, 
        FOCController& focController2, 
        Sensor& sensor1, 
        Sensor& sensor2, 
        InlineCurrentSense& currentSense1, 
        InlineCurrentSense& currentSense2, 
        LDM& ldm, 
        ScanPath& scanPath);

    ~System() = default;

    

    void loop();

}