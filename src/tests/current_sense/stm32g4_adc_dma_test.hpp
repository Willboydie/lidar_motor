// Stage 8 bench test — ADC hardware-triggered + DMA (docs/STM32G4_upgrade_stages.md).
//
// Motor Y: TIM8 master TRGO → ADC1 (PA1/A1 + PB0/A3), DMA1 Ch1.
// Streams one DualMotorStatusPacket (data in m2, m1 zero) for foc_plotter.py @ 460800.
//
// Packet fields (DualMotorStatusPacket — see Interface.hpp, normalised [-1,1]):
//   m2 ia, ib  <- measured phase currents A/B  (ic = -(ia+ib))
//   all other channels <- 0
// Raw 12-bit codes / pin volts are reported in the 1 Hz text summary only.
//
// Scope TRGO_PROBE_PIN (PB5 / D4) toggles on each ADC DMA sequence complete.
// (PC7 is motor Y PWM N2 — do not use as probe when running this test.)
//
// Self-sequenced phases (Serial.println announces each in the plotter log):
//   0-60 s   setPwm(0,0,0)           — zero-current bias (bench test 1)
//   60-120 s setPwm(6,0,0)           — PWM-modulation sync (bench test 4)
//   120 s+   sine Va on phase A      — 10 min soak stability (bench test 5)
// Bench tests 2 (scope rate) and 3 (polarity vs Stage 7) are manual anytime.

#pragma once

#include "Arduino.h"

#if defined(STM32G4xx)

#include "interface/Interface.hpp"
#include "current_sense/stm32g4/InlineCurrentSenseSTM32G4.hpp"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "pwm_driver/stm32g4/stm32_timerutils.h"
#include "utils/constants.h"
#include "utils/tweakable_params.h"

namespace {

constexpr uint32_t TRGO_PROBE_PIN = PB5;  // D4 — Y ConvCplt (PC7 = N2_Y PWM)
constexpr unsigned long kPhaseBiasMs = 60000;
constexpr unsigned long kPhaseFixedDutyMs = 120000;

enum class TestPhase : uint8_t {
    Bias = 0,
    FixedDuty = 1,
    SineSoak = 2,
};

const char* phaseName(TestPhase p) {
    switch (p) {
        case TestPhase::Bias: return "bias (Va=Vb=Vc=0)";
        case TestPhase::FixedDuty: return "fixed duty Va=6V (setPwm 6,0,0)";
        case TestPhase::SineSoak: return "sine Va soak (0.5 Hz, 3-9 V)";
        default: return "?";
    }
}

void applyPhase(PWMDriverSTM32G4& pwm, TestPhase phase, unsigned long t_ms) {
    switch (phase) {
        case TestPhase::Bias:
            pwm.setPwm(0.0f, 0.0f, 0.0f);
            break;
        case TestPhase::FixedDuty:
            pwm.setPwm(6.0f, 0.0f, 0.0f);
            break;
        case TestPhase::SineSoak: {
            const float t_s = t_ms * 0.001f;
            const float va = 6.0f + 3.0f * sinf(_2PI * 0.5f * t_s);
            pwm.setPwm(va, 0.0f, 0.0f);
            break;
        }
    }
}

TestPhase phaseForElapsed(unsigned long elapsed_ms) {
    if (elapsed_ms < kPhaseBiasMs) return TestPhase::Bias;
    if (elapsed_ms < kPhaseFixedDutyMs) return TestPhase::FixedDuty;
    return TestPhase::SineSoak;
}

}  // namespace

inline void stm32g4_adc_dma_test() {
    Serial.begin(BAUD_RATE);
    delay(2000);

    PWMDriverSTM32G4 pwmDriverY(N1_Y, N2_Y, N3_Y, EN_Y);
    InlineCurrentSenseSTM32G4 currentSense(
        ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, pwmDriverY);

    Serial.println("Stage 8: ADC TRGO+DMA test (motor Y) starting...");

    if (!pwmDriverY.init()) {
        Serial.println("Stage 8: PWM Y init FAILED");
        while (true) delay(1000);
    }
    pwmDriverY.enable();
    pwmDriverY.setPwm(0.0f, 0.0f, 0.0f);

    Serial.println(
        String("Stage 8: analogRead pre-init A1=") + analogRead(ADC_CS_Y_1)
        + " A3=" + analogRead(ADC_CS_Y_2)
    );

    TIM_TypeDef* master = pwmDriverY.getMasterTimerInstance();
    Serial.print("Stage 8: master TIM");
    Serial.print(stm32_getTimerNumber(master));
    Serial.print(" RCR=");
    Serial.println(master != NULL && IS_TIM_REPETITION_COUNTER_INSTANCE(master)
        ? static_cast<int>(master->RCR) : -1);

    currentSense.setAdcConvProbePin(TRGO_PROBE_PIN);
    if (!currentSense.init(true)) {
        Serial.println("Stage 8: current sense init FAILED");
        while (true) delay(1000);
    }

    uint16_t poll_a = 0, poll_b = 0;
    const bool poll_ok = currentSense.benchPollOnce(&poll_a, &poll_b);
    Serial.print("Stage 8: bench poll pre-DMA ");
    Serial.print(poll_ok ? "ok " : "FAIL ");
    Serial.print(poll_a);
    Serial.print(",");
    Serial.println(poll_b);

    currentSense.printHardwareDiagnostics();
    if (!currentSense.validateDmaSampling(50)) {
        Serial.println("Stage 8: ERR: DMA sampling validation FAILED");
        while (true) delay(1000);
    }

    Serial.println("Stage 8: probe pin PB5 (D4) toggles on ADC ConvCplt");
    Serial.println("Stage 8: streaming motor 2 at 200 Hz — foc_plotter @ 460800");

    const unsigned long testStartMs = millis();
    TestPhase activePhase = TestPhase::Bias;
    Serial.println(String("Stage 8 phase: ") + phaseName(activePhase));

    uint32_t n = 0;
    float sum_a = 0.0f, sumsq_a = 0.0f;
    float sum_b = 0.0f, sumsq_b = 0.0f;
    uint32_t conv_count_start = currentSense.getAdcConvCount();
    uint32_t tim_count_start = currentSense.getTimUpdateCount();

    unsigned long lastPacketTime = millis();
    unsigned long lastSummaryTime = millis();
    unsigned long lastPhaseApplyMs = millis();

    while (true) {
        const unsigned long elapsed = millis() - testStartMs;
        const TestPhase phase = phaseForElapsed(elapsed);
        if (phase != activePhase) {
            activePhase = phase;
            Serial.println(String("Stage 8 phase: ") + phaseName(activePhase));
        }

        if (millis() - lastPhaseApplyMs >= 10) {
            lastPhaseApplyMs = millis();
            applyPhase(pwmDriverY, activePhase, elapsed);
        }

        if (millis() - lastPacketTime >= FOC_STATUS_PERIOD_MS) {
            lastPacketTime = millis();

            uint16_t rawA, rawB;
            if (!currentSense.readRawCodes(rawA, rawB)) {
                Serial.println("Stage 8: ADC read failed");
                continue;
            }

            const float volts_a = rawA * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float volts_b = rawB * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float amps_a = (volts_a - currentSense.offset_ia) * currentSense.gain_a;
            const float amps_b = (volts_b - currentSense.offset_ib) * currentSense.gain_b;

            // Only motor 2 (Y) is sensed here; raw codes / pin volts stay in the
            // 1 Hz text summary. Phase C is reconstructed as -(A+B).
            const MotorStatus m2 = makeNormalisedMotorStatus(
                amps_a, amps_b, -(amps_a + amps_b), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            sendDualMotorStatusPacket(MotorStatus{}, m2);

            n++;
            sum_a += rawA;
            sumsq_a += (float)rawA * rawA;
            sum_b += rawB;
            sumsq_b += (float)rawB * rawB;
        }

        if (millis() - lastSummaryTime >= 1000 && n > 0) {
            lastSummaryTime = millis();

            const float mean_a = sum_a / n;
            const float mean_b = sum_b / n;
            const float std_a = sqrtf(max(0.0f, sumsq_a / n - mean_a * mean_a));
            const float std_b = sqrtf(max(0.0f, sumsq_b / n - mean_b * mean_b));
            const uint32_t conv_delta = currentSense.getAdcConvCount() - conv_count_start;
            conv_count_start = currentSense.getAdcConvCount();

            Serial.println(
                "Y: " + String(mean_a, 1) + " +/- " + String(std_a, 2) + " LSB ("
                + String(mean_a * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4) + " V) | B: "
                + String(mean_b, 1) + " +/- " + String(std_b, 2) + " LSB ("
                + String(mean_b * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4) + " V) | n="
                + String(n) + " | NDTR=" + String(currentSense.getDmaNdtr())
                + " OVR=" + String(currentSense.isAdcOverrun() ? 1 : 0)
                + " conv/s=" + String(conv_delta)
                + " tim/s=" + String(currentSense.getTimUpdateCount() - tim_count_start)
            );
            tim_count_start = currentSense.getTimUpdateCount();

            n = 0;
            sum_a = sumsq_a = 0.0f;
            sum_b = sumsq_b = 0.0f;
        }
    }
}

#endif  // STM32G4xx
