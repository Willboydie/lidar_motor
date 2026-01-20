#pragma once

#include "Arduino.h"
#include "../foc_controller/FOCController.hpp"
#include "../motor/Motor.hpp"
#include "../ldm/LDM.hpp"
#include "../scan_path/ScanPath.hpp"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"

#pragma pack(push, 1)
struct DataPacket {
    uint8_t cmd;
    uint16_t distance;
    float angle1;
    float angle2;
    float target1;
    float target2;
};
#pragma pack(pop)

class DataStream {
public:
    DataStream(FOCController& _focController1, FOCController& _focController2, LDM& _ldm, ScanPath& _scanPath);
    void sendData();

    FOCController& focController1;
    FOCController& focController2;
    LDM& ldm;
    ScanPath& scanPath;

private:
    float calculateAssociatedAngle(FOCController& focController);
};