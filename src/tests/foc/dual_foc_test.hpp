#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../interface/Interface.hpp"
#include "../../state_machine/StateMachine.hpp"
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../../scan_path/ScanPath.hpp"
#include "../../ldm/LDM.hpp"
#include "../../angle_tracker/AngleTracker.hpp"
#include "../../fault_manager/FaultManager.hpp"
#include "../wiring/wiring_common.hpp"

#if defined(STM32G4xx)
#include "../../current_sense/stm32g4/InlineCurrentSenseSTM32G4.hpp"
#include "../current_sense/current_sense_common.hpp"
#else
#include "../../current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#endif

// L15 — Dual-axis simultaneous FOC (final functional gate).
//
// Full application integration: both FOCControllers, StateMachine, Interface,
// and SinusoidalScanPath running together. Exercises enable/disable, scan
// play/pause, control-mode toggle, and e-stop; streams Interface binary
// telemetry plus human-readable DBG CSV.
//
// Pass bar (docs/FOC_bringup_plan.md L15):
//   - Both motors track the scan path simultaneously without interference.
//   - State machine + interface commands behave; e-stop disables both drives.
//   - Identical control behaviour on both targets (G4 current PI vs Teensy Vq model).
//
// Entry: L0–L14 passed for both X and Y on this target.

static constexpr uint32_t L15_THETA_INC = 150007u;   // ~10x slower than production THETA_INC
static constexpr uint32_t L15_PHI_INC   = 320003u;   // ~10x slower than production PHI_INC
static constexpr uint32_t L15_DBG_MS    = 100u;      // human CSV stream period [ms]

// Bench LDM stub — Interface expects an LDM; no Garmin hardware on the FOC bench.
class BenchStubLDM : public LDM {
public:
    BenchStubLDM() : LDM(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN) {}

    void init() override {
        markInitialised();
        Serial.println("LDM bench stub initialised (no hardware)");
    }

    void reconfigure(int config_num) override {
        Serial.print("LDM bench stub reconfigure ignored config=");
        Serial.println(config_num);
    }

private:
    void requestDistance() override {}
    void readLatestDistance() override { latest_distance_cm = 0; }
};

#if defined(STM32G4xx)
inline bool dual_foc_bringup_current_sense(
    PlatformPWMDriver& driver_x, InlineCurrentSenseSTM32G4& cs_x,
    PlatformPWMDriver& driver_y, InlineCurrentSenseSTM32G4& cs_y) {

    if (!driver_x.init() || !driver_y.init()) {
        Serial.println("RESULT dual_foc FAIL driver_init=0");
        return false;
    }
    if (!cs_x.init() || !cs_y.init()) {
        Serial.println("RESULT dual_foc FAIL current_sense_init=0");
        return false;
    }
    if (!cs_x.validateDmaSampling(50) || !cs_y.validateDmaSampling(50)) {
        Serial.println("RESULT dual_foc FAIL dma_sampling_invalid");
        return false;
    }

    driver_x.enable();
    driver_y.enable();
    driver_x.setPwm(0.0f, 0.0f, 0.0f);
    driver_y.setPwm(0.0f, 0.0f, 0.0f);
    delay(200);
    cs_x.calibrateOffsets(1000);
    cs_y.calibrateOffsets(1000);
    driver_x.disable();
    driver_y.disable();

    Serial.print("CAL dual_foc X offset_ia_V=");
    Serial.print(cs_x.offset_ia, 4);
    Serial.print(" offset_ib_V=");
    Serial.print(cs_x.offset_ib, 4);
    Serial.print(" Y offset_ia_V=");
    Serial.print(cs_y.offset_ia, 4);
    Serial.print(" offset_ib_V=");
    Serial.println(cs_y.offset_ib, 4);
    return true;
}
#endif

inline void dual_foc_print_spec() {
    Serial.print("SPEC theta_inc=");
    Serial.print(L15_THETA_INC);
    Serial.print(" phi_inc=");
    Serial.print(L15_PHI_INC);
    Serial.print(" theta_amp_rad=");
    Serial.print(THETA_AMPLITUDE, 3);
    Serial.print(" phi_amp_rad=");
    Serial.print(PHI_AMPLITUDE, 3);
    Serial.print(" theta_centre_rad=");
    Serial.print(THETA_CENTRE, 3);
    Serial.print(" phi_centre_rad=");
    Serial.println(PHI_CENTRE, 3);
    Serial.print("SPEC loop_Hz current=");
    Serial.print(CURRENT_LOOP_HZ);
    Serial.print(" velocity=");
    Serial.print(VELOCITY_LOOP_HZ);
    Serial.print(" angle=");
    Serial.print(ANGLE_LOOP_HZ);
    Serial.print(" bin_status_ms=");
    Serial.println(FOC_STATUS_PERIOD_MS);
}

inline void dual_foc_bench_prompt() {
    Serial.println();
    Serial.println("SAFETY — L15 dual-axis FOC");
    Serial.println("  BOTH rotors FREE and UNLOADED (>=1 turn clearance each).");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.5-2.0 A (both axes).");
    Serial.println("  E-stop: ' '/'x' (disables both via Interface). Cut VM if needed.");
    dual_foc_print_spec();
    Serial.println("CONTROLS: 'e'=enable  'd'=disable  's'=scan play/pause  'm'=toggle ANGLE/VELOCITY");
    Serial.println("          'h'=hold at current angles  ' '/'x'=E-STOP  'q'=end test.");
    Serial.println("BINARY: Interface accepts 5-byte UI packets (cmd+float) as in production.");
    wiring_wait_for_confirm("both rotors free/unloaded, VM on + current-limited for dual-axis");
}

inline const char* dual_foc_mode_name(ControlMode mode) {
    switch (mode) {
        case ANGLE: return "ANGLE";
        case VELOCITY: return "VELOCITY";
        case PATH: return "PATH";
        default: return "?";
    }
}

inline void dual_foc_hold_positions(FOCController& foc_x, FOCController& foc_y) {
    foc_x.setTargetAngle(foc_x.getMechanicalAngle());
    foc_y.setTargetAngle(foc_y.getMechanicalAngle());
    Serial.println("CMD hold — targets set to current mechanical angles");
}

// L15 — Dual-axis simultaneous FOC. Both stm32g4 and teensy41.
inline void dual_foc_test() {
    wiring_serial_begin();

    Serial.print("TEST dual_foc axis=dual target=");
    Serial.println(wiring_target_name());
    Serial.println("L15 — dual-axis simultaneous FOC (StateMachine + Interface + SinusoidalScanPath).");
#if defined(STM32G4xx)
    Serial.println("STM32G4: inner loop = measured-current PI (L11/L12).");
#elif defined(ARDUINO_TEENSY41)
    Serial.println("Teensy: inner loop = estimate_Vq voltage model (skips L11/L12).");
#endif

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver_x(N1_X, N2_X, N3_X, EN_X);
    PlatformPWMDriver driver_y(N1_Y, N2_Y, N3_Y, EN_Y);

#if defined(STM32G4xx)
    InlineCurrentSenseSTM32G4 cs_x(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, driver_x);
    InlineCurrentSenseSTM32G4 cs_y(ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, driver_y);
    if (!dual_foc_bringup_current_sense(driver_x, cs_x, driver_y, cs_y)) {
        while (true) { delay(1000); }
    }
#else
    if (!driver_x.init() || !driver_y.init()) {
        Serial.println("RESULT dual_foc FAIL driver_init=0");
        while (true) { delay(1000); }
    }
#endif

    GM2804 motor_x(1, SENSOR_OFFSET_X);
    GM2804 motor_y(2, SENSOR_OFFSET_Y);
    AS5048 sensor_x(SENSOR_SPI_CS_X);
    AS5048 sensor_y(SENSOR_SPI_CS_Y);
    FOCController foc_x(sensor_x, driver_x, motor_x);
    FOCController foc_y(sensor_y, driver_y, motor_y);

#if defined(STM32G4xx)
    foc_x.setCurrentSense(cs_x);
    foc_y.setCurrentSense(cs_y);
#endif

    if (!sensor_x.init() || !sensor_y.init()) {
        Serial.println("RESULT dual_foc FAIL encoder_init=0");
        while (true) { delay(1000); }
    }
    sensor_x.enable();
    sensor_y.enable();

    if (!foc_x.init() || !foc_y.init()) {
        Serial.println("RESULT dual_foc FAIL foc_init=0");
        while (true) { delay(1000); }
    }

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::alignAllRegisteredSubmodules();
    Serial.println("SYNC dual teensy_submodule_align=done");
#endif

    foc_x.setControlMode(ANGLE);
    foc_y.setControlMode(ANGLE);
    dual_foc_hold_positions(foc_x, foc_y);

    StateMachine state_machine;
    BenchStubLDM ldm;
    ldm.init();
    AngleTracker angle_tracker_x(sensor_x, faultManager, FAULT_TRACKER_X_OVERFLOW);
    AngleTracker angle_tracker_y(sensor_y, faultManager, FAULT_TRACKER_Y_OVERFLOW);
    SinusoidalScanPath scan_path(
        L15_THETA_INC, L15_PHI_INC,
        THETA_AMPLITUDE, PHI_AMPLITUDE,
        THETA_CENTRE, PHI_CENTRE);
#if defined(STM32G4xx)
    SystemManager system_manager(foc_x, foc_y, sensor_x, sensor_y, cs_x, cs_y, ldm,
                                 driver_x, driver_y, faultManager, state_machine);
    Interface interface(state_machine, foc_x, foc_y, sensor_x, sensor_y, cs_x, cs_y, ldm,
                        angle_tracker_x, angle_tracker_y, scan_path, faultManager, system_manager);
#else
    InlineCurrentSenseTeensy41 cs_stub_x;
    InlineCurrentSenseTeensy41 cs_stub_y;
    SystemManager system_manager(foc_x, foc_y, sensor_x, sensor_y, cs_stub_x, cs_stub_y, ldm,
                                 driver_x, driver_y, faultManager, state_machine);
    Interface interface(state_machine, foc_x, foc_y, sensor_x, sensor_y, cs_stub_x, cs_stub_y, ldm,
                        angle_tracker_x, angle_tracker_y, scan_path, faultManager, system_manager);
#endif

    // Mark SystemManager ready so Interface enable/disable paths work. Sensors /
    // PWM / FOC are already initialised above; SystemManager::init is idempotent
    // for those and also initialises the bench LDM stub.
    system_manager.init();
    if (!system_manager.isInitialised()) {
        Serial.println("RESULT dual_foc FAIL system_manager_init=0");
        while (true) { delay(1000); }
    }

    dual_foc_bench_prompt();

    Serial.println("MOD phase=dual_foc — both axes integrated; enable then scan.");
    Serial.println("DBG columns: t_ms,en,scan,mode,tgt_x,mech_x,err_x,tgt_y,mech_y,err_y");

    uint32_t last_dbg_ms = millis();
    uint32_t last_bin_ms = millis();

    while (true) {
        interface.commandListener();

        if (state_machine.enabled) {
            if (state_machine.scanning && state_machine.controlMode == ANGLE) {
                scan_path.update();
                foc_x.setTargetAngle(scan_path.theta);
                foc_y.setTargetAngle(scan_path.phi);
            }

            if (!foc_x.loop() || !foc_y.loop()) {
                interface.processCommand(DISABLE_ENABLE_CMD, 0.0f);
                state_machine.scanning = false;
                Serial.println("RESULT dual_foc FAIL loop=0");
            }
        }

        const uint32_t now_ms = millis();

        // if (state_machine.enabled && (now_ms - last_bin_ms >= FOC_STATUS_PERIOD_MS)) {
        //     last_bin_ms = now_ms;
        //     interface.sendDualMotorStatus();
        // }

        if (now_ms - last_dbg_ms >= L15_DBG_MS) {
            last_dbg_ms = now_ms;
            Serial.print("DBG ");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(state_machine.enabled ? 1 : 0);
            Serial.print(",");
            Serial.print(state_machine.scanning ? 1 : 0);
            Serial.print(",");
            Serial.print(dual_foc_mode_name(state_machine.controlMode));
            Serial.print(",");
            Serial.print(foc_x.getTargetAngle(), 4);
            Serial.print(",");
            Serial.print(foc_x.getMechanicalAngle(), 4);
            Serial.print(",");
            Serial.print(foc_x.getAngularError(), 4);
            Serial.print(",");
            Serial.print(foc_y.getTargetAngle(), 4);
            Serial.print(",");
            Serial.print(foc_y.getMechanicalAngle(), 4);
            Serial.print(",");
            Serial.println(foc_y.getAngularError(), 4);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == 'e' || c == 'E') {
                if (!state_machine.enabled) {
                    dual_foc_hold_positions(foc_x, foc_y);
                }
                interface.processCommand(DISABLE_ENABLE_CMD, 0.0f);
                Serial.print("CMD enable=");
                Serial.println(state_machine.enabled ? 1 : 0);
            } else if (c == 'd' || c == 'D') {
                if (state_machine.enabled) {
                    interface.processCommand(DISABLE_ENABLE_CMD, 0.0f);
                }
                Serial.print("CMD enable=");
                Serial.println(state_machine.enabled ? 1 : 0);
            } else if (c == 's' || c == 'S') {
                interface.processCommand(SCAN_PLAY_PAUSE_CMD, 0.0f);
                Serial.print("CMD scanning=");
                Serial.println(state_machine.scanning ? 1 : 0);
            } else if (c == 'm' || c == 'M') {
                interface.processCommand(TOGGLE_CONTROL_MODE_CMD, 0.0f);
                Serial.print("CMD mode=");
                Serial.println(dual_foc_mode_name(state_machine.controlMode));
            } else if (c == 'h' || c == 'H') {
                state_machine.scanning = false;
                dual_foc_hold_positions(foc_x, foc_y);
            } else if (c == ' ' || c == 'x' || c == 'X') {
                if (state_machine.enabled) {
                    interface.processCommand(DISABLE_ENABLE_CMD, 0.0f);
                }
                state_machine.scanning = false;
                Serial.println("ESTOP");
            } else if (c == 'q' || c == 'Q') {
                if (state_machine.enabled) {
                    interface.processCommand(DISABLE_ENABLE_CMD, 0.0f);
                }
                break;
            }
        }
    }

    foc_x.deinit();
    foc_y.deinit();
    driver_x.deinit();
    driver_y.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.print("RESULT dual_foc CHECK target=");
    Serial.print(wiring_target_name());
    Serial.println(" (operator confirms: both track scan path, no cross-axis fault, e-stop OK)");

    while (true) { delay(1000); }
}
