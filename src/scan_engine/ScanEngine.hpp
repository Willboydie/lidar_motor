#pragma once

#include "../foc_controller/FOCController.hpp"
#include "coarse/CoarseScan.hpp"
#include "s_curve/SCurve.hpp"
#include "../utils/quick_defs.h"

enum class ScanMode {
    OFF = 0,
    COARSE = 1,
    POINT2POINT = 2,
};

enum class DynamicRoutine {
    NONE = 0,
    WINDING_UP_TO_COARSE = 1,
    TAPERING_DOWN_FROM_COARSE = 2,
};

class ScanEngine {

public:
    ScanEngine(FOCController& _focControllerX, FOCController& _focControllerY, CoarseScan& _coarseScan, SCurve& _sCurveX, SCurve& _sCurveY);
    ~ScanEngine() = default;

    void on();
    void off();
    void seekNeutral();
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    void beginCoarseScanning();
    void stopCoarseScanning();

    void updatePosition(float dt);
    void updateVelocity(float dt);

    void setNewPoint2PointTarget(float _angleX, float _angleY);
    void updateCoarseScanCentre(float _coarseScanCentreX, float _coarseScanCentreY);

    float getCoarseScanCentreX() { return baseAngleX; }
    float getCoarseScanCentreY() { return baseAngleY; }
    float getPoint2PointTargetX() { return adaptiveTargetX; }
    float getPoint2PointTargetY() { return adaptiveTargetY; }

    DynamicRoutine getDynamicRoutine() { return dynamicRoutine; }
    ScanMode getScanMode() { return scanMode; }

private:
    FOCController& focControllerX;
    FOCController& focControllerY;
    CoarseScan& coarseScan;
    SCurve& sCurveX;
    SCurve& sCurveY;

    bool enabled = false;

    void dynamicRoutineUpdate();

    void windUpToCoarse();
    void taperDownFromCoarse();
    void establishAsPoint2PointBase();

    ScanMode scanMode = ScanMode::OFF;
    DynamicRoutine dynamicRoutine = DynamicRoutine::NONE;
    float dilationTerm = 1.0f;

    float baseAngleX = DEFAULT_NEUTRAL_X_POS;
    float baseAngleY = DEFAULT_NEUTRAL_Y_POS;
    float dx = 0.0f;
    float dy = 0.0f;
    float velocityX = 0.0f;
    float velocityY = 0.0f;
    float adaptiveTargetX = 0.0f;
    float adaptiveTargetY = 0.0f;
    float checkpointAngleX = 0.0f;
    float checkpointAngleY = 0.0f;

    const float neutralX = DEFAULT_NEUTRAL_X_POS;
    const float neutralY = DEFAULT_NEUTRAL_Y_POS;
};