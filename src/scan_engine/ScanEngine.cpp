#include "ScanEngine.hpp"
#include "../debug/Debug.hpp"

ScanEngine::ScanEngine(FOCController& _focControllerX, FOCController& _focControllerY, CoarseScan& _coarseScan, SCurve& _sCurveX, SCurve& _sCurveY)
: focControllerX(_focControllerX)
, focControllerY(_focControllerY)
, coarseScan(_coarseScan)
, sCurveX(_sCurveX)
, sCurveY(_sCurveY) {}


// ------------------- High Control --------------------------
void ScanEngine::on() {
    if (scanMode == ScanMode::OFF) {
        scanMode = ScanMode::POINT2POINT;
        seekNeutral();
        Debug::msg("ScanEngine turned on.");
    } else {
        Debug::msg("ScanEngine already on.");
    }
}

void ScanEngine::off() {
    if (scanMode != ScanMode::OFF) {
        scanMode = ScanMode::OFF;
        dynamicRoutine = DynamicRoutine::NONE;
        coarseScan.resetTime();
        Debug::msg("ScanEngine turned off.");
    } else {
        Debug::msg("ScanEngine already off.");
    }
}

void ScanEngine::beginCoarseScanning() {
    if (scanMode == ScanMode::OFF) {
        Debug::msg("Turn ScanEngine on to begin coarse scanning.");
        return;
    }
    
    if (!coarseScan.xCentreWithinLimits(focControllerX.getMechanicalAngle()) || !coarseScan.yCentreWithinLimits(focControllerY.getMechanicalAngle())) {
        Debug::msg("Coarse scan centre outwith bounds.");
        return;
    } 

    establishAsPoint2PointBase();
    coarseScan.resetTime();
    scanMode = ScanMode::COARSE;
    dynamicRoutine = DynamicRoutine::WINDING_UP_TO_COARSE;
    Debug::msg("Beginning coarse windup.");
}

void ScanEngine::stopCoarseScanning() {
    if (scanMode != ScanMode::COARSE) {
        return;
    }
    dynamicRoutine = DynamicRoutine::TAPERING_DOWN_FROM_COARSE;
}

void ScanEngine::seekNeutral() {
    if (scanMode != ScanMode::POINT2POINT) {
        return;
    }
    setNewPoint2PointTarget(neutralX, neutralY);
}


// ------------------- Position and Velocity Updates --------------------------
void ScanEngine::updatePosition(float dt) {
    if (!enabled) return;
    dynamicRoutineUpdate();

    switch (scanMode) {
        case ScanMode::COARSE:
            coarseScan.updateXPosition(dilationTerm * dt, dx);
            coarseScan.updateYPosition(dilationTerm * dt, dy);
            break;
        case ScanMode::POINT2POINT:
            sCurveX.updatePosition(dt, dx);
            sCurveY.updatePosition(dt, dy);
            break;
        case ScanMode::OFF:
            break;
    }
    focControllerX.setTargetAngle(baseAngleX + dx);
    focControllerY.setTargetAngle(baseAngleY + dy);
}

void ScanEngine::updateVelocity(float dt) {
    if (!enabled) return;
    switch (scanMode) {
        case ScanMode::COARSE:
            coarseScan.updateXVelocity(dilationTerm * dt, velocityX);
            coarseScan.updateYVelocity(dilationTerm * dt, velocityY);
            break;
        case ScanMode::POINT2POINT:
            sCurveX.updateVelocity(dt, velocityX);
            sCurveY.updateVelocity(dt, velocityY);
            break;
        case ScanMode::OFF:
            break;
    }
    focControllerX.setApproachVelocity(velocityX);
    focControllerY.setApproachVelocity(velocityY);
}

// ------------------- Dynamic Routine Management --------------------------------
void ScanEngine::dynamicRoutineUpdate() {
    switch (dynamicRoutine) {

        case DynamicRoutine::WINDING_UP_TO_COARSE:
            windUpToCoarse();
            break;

        case DynamicRoutine::TAPERING_DOWN_FROM_COARSE:
            taperDownFromCoarse();
            break;

        case DynamicRoutine::NONE:
            break;

        default:
            break;
    }
}

// ------------------- dynamic flow helpers --------------------------

void ScanEngine::taperDownFromCoarse() {
    dilationTerm *= 0.999f;
    if (dilationTerm < 0.005f) {
        dilationTerm = 0.005f;
        Debug::msg("Dilation term reached 0.005.");
        scanMode = ScanMode::POINT2POINT;
        dynamicRoutine = DynamicRoutine::NONE;
        seekNeutral();
    }
}

void ScanEngine::windUpToCoarse() {
    dilationTerm *= 1.001f;
    if (dilationTerm > 1.0f) {
        dilationTerm = 1.0f;
        Debug::msg("Dilation term reached 1.0.");
        dynamicRoutine = DynamicRoutine::NONE;
    }
}


// -------------------- Control Setters --------------------------

void ScanEngine::establishAsPoint2PointBase() {
    baseAngleX = focControllerX.getMechanicalAngle();
    baseAngleY = focControllerY.getMechanicalAngle();
    sCurveX.setTimeToEnd();
    sCurveY.setTimeToEnd();
}

void ScanEngine::setNewPoint2PointTarget(float _angleX, float _angleY) {
    if (scanMode != ScanMode::POINT2POINT) {
        Debug::msg("Cannot set point2point target when not in point2point mode.");
        return;
    }
    if (!isWithinXAngleLimits(_angleX) || !isWithinYAngleLimits(_angleY)) {
        Debug::msg("Point2Point target outwith bounds.");
        return;
    }
    baseAngleX = focControllerX.getMechanicalAngle();
    baseAngleY = focControllerY.getMechanicalAngle();
    adaptiveTargetX = _angleX;
    adaptiveTargetY = _angleY;
    float Dx = adaptiveTargetX - baseAngleX;
    float Dy = adaptiveTargetY - baseAngleY;
    sCurveX.setNewDisplacement(Dx);
    sCurveY.setNewDisplacement(Dy);
}

void ScanEngine::updateCoarseScanCentre(float _centreX, float _centreY) {
    if (scanMode != ScanMode::COARSE) {
        Debug::msg("Cannot update coarse scan centre when not in coarse mode.");
        return;
    }
    if (coarseScan.xCentreWithinLimits(_centreX) && coarseScan.yCentreWithinLimits(_centreY)) {
        baseAngleX = _centreX;
        baseAngleY = _centreY;
    } else {
        Debug::msg("Coarse scan centre outwith bounds (or limit function not implemented)");
    }
}