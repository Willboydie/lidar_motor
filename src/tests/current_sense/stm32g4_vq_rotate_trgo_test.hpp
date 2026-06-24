// Stage 8.6 bench test — open-loop Vq rotation + centre-aligned PWM + TRGO timing
// (docs/STM32G4_upgrade_stages.md).
//
// Spins a constant Vq vector around both motors by ramping electrical angle (inverse
// Park + Clarke → setPwm). No ADC / current sense — this is the PWM+TRGO scope gate
// before Stage 9's Iq open-loop test.
//
// Scope checklist (centre-aligned PWM1, TRGO = UPDATE, RCR = 1):
//   Motor X PWM:  PA8 (D7), PA9 (D8), PA10 (D2)   — TIM1
//   Motor Y PWM:  PC6 (D47), PC7 (D9), PB9 (D14)  — TIM8
//   TRGO proxy X: PA6 (D12) — toggles on TIM1 UPDATE (same instant as TRGO)
//   TRGO proxy Y: PB5 (D4)  — toggles on TIM8 UPDATE
//
// Pass on scope:
//   1. All six PWM outputs show smooth sinusoidal duty modulation as angle advances.
//   2. TRGO proxy frequency = PWM freq (25 kHz) with RCR=1.
//   3. TRGO edge sits at the counter turnaround (triangle bottom / count-zero) — the
//      fixed sample phase used for ADC sync ("mid-on" in docs/STM32G4_pwm_and_current_sense.md).
//      Compare one PWM channel (e.g. PA8) with PA6 on the same timebase: TRGO should
//      land at the same sub-cycle position every carrier period, not walk with duty.
//
// foc_plotter @ 460800: Angle + Requested Vq + Va/Vb/Vc on both motors (dual overlay).

#pragma once

#include "Arduino.h"

#if defined(STM32G4xx)

#include "interface/Interface.hpp"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "pwm_driver/stm32g4/stm32_timerutils.h"
#include "utils/constants.h"
#include "utils/quick_defs.h"
#include "utils/tweakable_params.h"

namespace {

constexpr uint32_t TRGO_PROBE_X_PIN = PA6;  // D12
constexpr uint32_t TRGO_PROBE_Y_PIN = PB5;  // D4

constexpr float kVq = 4.0f;
constexpr float kVd = 0.0f;
// One electrical revolution every 10 s — slow enough to watch PWM morph on a scope.
constexpr float kElecRevPeriodSec = 10.0f;
constexpr float kElecRadPerSec = _2PI / kElecRevPeriodSec;
constexpr unsigned long kVoltageUpdateUs = 200;  // 5 kHz open-loop update

TIM_HandleTypeDef* g_master_x = NULL;
TIM_HandleTypeDef* g_master_y = NULL;
volatile uint32_t g_trgo_x_count = 0;
volatile uint32_t g_trgo_y_count = 0;

void enable_trgo_probes(TIM_HandleTypeDef* master_x, TIM_HandleTypeDef* master_y) {
    g_master_x = master_x;
    g_master_y = master_y;
    pinMode(TRGO_PROBE_X_PIN, OUTPUT);
    pinMode(TRGO_PROBE_Y_PIN, OUTPUT);
    digitalWrite(TRGO_PROBE_X_PIN, LOW);
    digitalWrite(TRGO_PROBE_Y_PIN, LOW);
}

void poll_trgo_probes() {
    if (g_master_x != NULL && __HAL_TIM_GET_FLAG(g_master_x, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(g_master_x, TIM_FLAG_UPDATE);
        digitalWrite(TRGO_PROBE_X_PIN, digitalRead(TRGO_PROBE_X_PIN) ? LOW : HIGH);
        g_trgo_x_count++;
    }
    if (g_master_y != NULL && __HAL_TIM_GET_FLAG(g_master_y, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(g_master_y, TIM_FLAG_UPDATE);
        digitalWrite(TRGO_PROBE_Y_PIN, digitalRead(TRGO_PROBE_Y_PIN) ? LOW : HIGH);
        g_trgo_y_count++;
    }
}

// Mirror of FOCController::setPhaseVoltage — no sensor / FOC loop required.
void applyOpenLoopVq(PWMDriverSTM32G4& pwm, float elec_angle, float Vq, float Vd) {
    float sa = 0.0f;
    float ca = 0.0f;
    fast_sincos(elec_angle, &sa, &ca);

    const float Valpha = ca * Vd - sa * Vq;
    const float Vbeta = sa * Vd + ca * Vq;

    float Va = Valpha;
    float Vb = -0.5f * Valpha + _SQRT3_2 * Vbeta;
    float Vc = -0.5f * Valpha - _SQRT3_2 * Vbeta;

    float center = pwm.getVoltageLimit() * 0.5f;
    const float Vmin = min(Va, min(Vb, Vc));
    const float Vmax = max(Va, max(Vb, Vc));
    center -= (Vmax + Vmin) * 0.5f;

    pwm.setPwm(Va + center, Vb + center, Vc + center);
}

void streamDualPackets(float angle_x, float Va_x, float Vb_x, float Vc_x,
                       float angle_y, float Va_y, float Vb_y, float Vc_y) {
    // No current sensing in this test — ABC/DQ channels stream as zero.
    // The angle here is the commanded electrical angle (no sensor read).
    const MotorStatus m1 = makeNormalisedMotorStatus(
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, angle_x, angle_x, 0.0f, Va_x, Vb_x, Vc_x);
    const MotorStatus m2 = makeNormalisedMotorStatus(
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, angle_y, angle_y, 0.0f, Va_y, Vb_y, Vc_y);
    sendDualMotorStatusPacket(m1, m2);
}

void print_master_timer(const char* label, TIM_TypeDef* tim) {
    Serial.print(label);
    Serial.print(" master TIM");
#if defined(SIMPLEFOC_STM32_DEBUG)
    Serial.print(stm32_getTimerNumber(tim));
#else
    Serial.print(reinterpret_cast<uintptr_t>(tim), HEX);
#endif
    if (tim != NULL && IS_TIM_REPETITION_COUNTER_INSTANCE(tim)) {
        Serial.print(" RCR=");
        Serial.print(static_cast<int>(tim->RCR));
    }
    const uint32_t trgo = (tim != NULL) ? ((tim->CR2 >> TIM_CR2_MMS_Pos) & 0x7U) : 0;
    Serial.print(" TRGO=");
    Serial.println(trgo == LL_TIM_TRGO_UPDATE ? "UPDATE" : "other");
}

}  // namespace

inline void stm32g4_vq_rotate_trgo_test() {
    Serial.begin(BAUD_RATE);
    delay(2000);

    PWMDriverSTM32G4 pwmDriverX(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 pwmDriverY(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.println("Stage 8.6: Vq rotate + centre-aligned PWM + TRGO scope test");

    if (!pwmDriverX.init()) {
        Serial.println("Stage 8.6: PWM X init FAILED");
        while (true) delay(1000);
    }
    if (!pwmDriverY.init()) {
        Serial.println("Stage 8.6: PWM Y init FAILED");
        while (true) delay(1000);
    }
    pwmDriverX.enable();
    pwmDriverY.enable();

    TIM_HandleTypeDef* syncMasters[2] = {
        pwmDriverX.getMasterTimerHandle(),
        pwmDriverY.getMasterTimerHandle(),
    };
    stm32_syncTimers(syncMasters, 2);

    print_master_timer("X:", pwmDriverX.getMasterTimerInstance());
    print_master_timer("Y:", pwmDriverY.getMasterTimerInstance());

    enable_trgo_probes(pwmDriverX.getMasterTimerHandle(), pwmDriverY.getMasterTimerHandle());
    g_trgo_x_count = 0;
    g_trgo_y_count = 0;

    Serial.println(String("Stage 8.6: Vq=") + kVq + " V, elec ramp "
        + String(kElecRadPerSec, 4) + " rad/s (1 rev / "
        + String(kElecRevPeriodSec, 1) + " s)");
    Serial.println("Stage 8.6: PWM X = PA8/PA9/PA10  Y = PC6/PC7/PB9");
    Serial.println("Stage 8.6: TRGO proxy X = PA6 (D12)  Y = PB5 (D4) on TIM UPDATE");
    Serial.println("Stage 8.6: foc_plotter — Raw Angle + Va/Vb/Vc on both motors");
    Serial.println("Stage 8.6: scope TRGO vs PWM — edge at triangle bottom, fixed phase each period");

    float elec_angle = 0.0f;
    float Va_x = 0.0f, Vb_x = 0.0f, Vc_x = 0.0f;
    float Va_y = 0.0f, Vb_y = 0.0f, Vc_y = 0.0f;

    unsigned long lastVoltageUs = micros();
    unsigned long lastPacketMs = millis();
    unsigned long lastSummaryMs = millis();
    uint32_t trgo_x_start = g_trgo_x_count;
    uint32_t trgo_y_start = g_trgo_y_count;

    while (true) {
        poll_trgo_probes();

        const unsigned long nowUs = micros();
        if ((unsigned long)(nowUs - lastVoltageUs) >= kVoltageUpdateUs) {
            const float dt = (nowUs - lastVoltageUs) * 1e-6f;
            lastVoltageUs = nowUs;

            elec_angle = normalise_angle(elec_angle + kElecRadPerSec * dt);
            applyOpenLoopVq(pwmDriverX, elec_angle, kVq, kVd);
            applyOpenLoopVq(pwmDriverY, elec_angle, kVq, kVd);

            float sa = 0.0f, ca = 0.0f;
            fast_sincos(elec_angle, &sa, &ca);
            const float Valpha = ca * kVd - sa * kVq;
            const float Vbeta = sa * kVd + ca * kVq;
            Va_x = Va_y = Valpha;
            Vb_x = Vb_y = -0.5f * Valpha + _SQRT3_2 * Vbeta;
            Vc_x = Vc_y = -0.5f * Valpha - _SQRT3_2 * Vbeta;
        }

        if (millis() - lastPacketMs >= FOC_STATUS_PERIOD_MS) {
            lastPacketMs = millis();
            streamDualPackets(
                elec_angle, Va_x, Vb_x, Vc_x,
                elec_angle, Va_y, Vb_y, Vc_y
            );
        }

        if (millis() - lastSummaryMs >= 1000) {
            lastSummaryMs = millis();
            const uint32_t trgo_x_delta = g_trgo_x_count - trgo_x_start;
            const uint32_t trgo_y_delta = g_trgo_y_count - trgo_y_start;
            trgo_x_start = g_trgo_x_count;
            trgo_y_start = g_trgo_y_count;
            Serial.print("Stage 8.6: angle=");
            Serial.print(elec_angle, 3);
            Serial.print(" rad | TRGO X=");
            Serial.print(trgo_x_delta);
            Serial.print("/s Y=");
            Serial.print(trgo_y_delta);
            Serial.println("/s (expect ~25000/s with RCR=1 @ 25 kHz PWM)");
        }
    }
}

#endif  // STM32G4xx
