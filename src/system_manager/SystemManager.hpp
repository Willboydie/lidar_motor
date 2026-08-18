#pragma once

#include "foc_controller/FOCController.hpp"
#include "sensor/Sensor.hpp"
#include "current_sense/InlineCurrentSense.hpp"
#include "ldm/LDM.hpp"
#include "pwm_driver/PWMDriver.hpp"
#include "fault_manager/FaultManager.hpp"
#include "state_machine/StateMachine.hpp"
#include "scan_engine/ScanEngine.hpp"
#include "../debug/Debug.hpp"

class SystemManager {
public:
    SystemManager(FOCController& _focControllerX, FOCController& _focControllerY, Sensor& _sensorX, Sensor& _sensorY, InlineCurrentSense& _currentSenseX, InlineCurrentSense& _currentSenseY, LDM& _ldm, PWMDriver& _pwmDriverX, PWMDriver& _pwmDriverY, FaultManager& _faultManager, StateMachine& _stateMachine, ScanEngine& _scanEngine);
    ~SystemManager() = default;

    void init();
    void enable();
    void disable();
    void restart();
    void sleep();
    void wake();
    void enterBootloader();

    bool isInitialised() { return initialised; }
    bool isEnabled() { return enabled; }

private:
    FOCController& focControllerX;
    FOCController& focControllerY;
    Sensor& sensorX;
    Sensor& sensorY;
    InlineCurrentSense& currentSenseX;
    InlineCurrentSense& currentSenseY;
    LDM& ldm;
    PWMDriver& pwmDriverX;
    PWMDriver& pwmDriverY;
    FaultManager& faultManager;
    StateMachine& stateMachine;
    ScanEngine& scanEngine;

    bool initialised = false;
    bool enabled = false;
};
