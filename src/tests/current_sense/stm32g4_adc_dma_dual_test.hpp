// Stage 8.5 bench test — ADC TRGO+DMA on BOTH motors (docs/STM32G4_upgrade_stages.md).
//
// Same self-sequenced phases as Stage 8, but motor X and motor Y run in parallel:
// both PWM drivers + both InlineCurrentSenseSTM32G4 instances (X: TIM1/ADC2, Y: TIM8/ADC1).
// Streams FOCStatusPackets for motor 1 and motor 2 at 200 Hz each.
//
// Packet fields (FOCStatusPacket — see Interface.hpp):
//   raw_adc_a, raw_adc_b  <- raw 12-bit codes [LSB]
//   amps_a, amps_b        <- nominal phase currents [A]
//   volts_a, volts_b      <- pin voltages [V]
//   all motion / Iq fields <- 0
//
// Scope probes (ADC ConvCplt toggle):
//   X: PA6 (D12)   Y: PB5 (D4) — same pins as Stage 4 dual TRGO test.
//
// Self-sequenced phases (Serial.println announces each in the plotter log):
//   0-60 s   setPwm(0,0,0) on both     — zero-current bias (bench test 1)
//   60-120 s setPwm(6,0,0) on both     — PWM-modulation sync (bench test 4)
//   120 s+   sine Va on phase A both   — 10 min soak stability (bench test 5)
// Bench tests 2 (scope rate) and 3 (polarity vs Stage 7) are manual anytime.
// Switch foc_plotter motor selector between 1 (X) and 2 (Y) to inspect each trace.

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

constexpr uint32_t TRGO_PROBE_X_PIN = PA6;  // D12
constexpr uint32_t TRGO_PROBE_Y_PIN = PB5;  // D4
constexpr unsigned long kPhaseBiasMs = 6000;
constexpr unsigned long kPhaseFixedDutyMs = 12000;

enum class TestPhase : uint8_t {
    Bias = 0,
    FixedDuty = 1,
    SineSoak = 2,
};

const char* phaseName(TestPhase p) {
    switch (p) {
        case TestPhase::Bias: return "bias (Va=Vb=Vc=0, both motors)";
        case TestPhase::FixedDuty: return "fixed duty Va=6V (setPwm 6,0,0, both)";
        case TestPhase::SineSoak: return "sine Va soak (0.5 Hz, 3-9 V, both)";
        default: return "?";
    }
}

void applyPhase(PWMDriverSTM32G4& pwmX, PWMDriverSTM32G4& pwmY, TestPhase phase, unsigned long t_ms) {
    switch (phase) {
        case TestPhase::Bias:
            pwmX.setPwm(0.0f, 0.0f, 0.0f);
            pwmY.setPwm(0.0f, 0.0f, 0.0f);
            break;
        case TestPhase::FixedDuty:
            pwmX.setPwm(6.0f, 0.0f, 0.0f);
            pwmY.setPwm(6.0f, 0.0f, 0.0f);
            break;
        case TestPhase::SineSoak: {
            const float t_s = t_ms * 0.001f;
            const float va = 6.0f + 3.0f * sinf(_2PI * 0.5f * t_s);
            pwmX.setPwm(va, 0.0f, 0.0f);
            pwmY.setPwm(va, 0.0f, 0.0f);
            break;
        }
    }
}

TestPhase phaseForElapsed(unsigned long elapsed_ms) {
    if (elapsed_ms < kPhaseBiasMs) return TestPhase::Bias;
    if (elapsed_ms < kPhaseFixedDutyMs) return TestPhase::FixedDuty;
    return TestPhase::SineSoak;
}

struct ChannelStats {
    uint32_t n = 0;
    float sum_a = 0.0f;
    float sumsq_a = 0.0f;
    float sum_b = 0.0f;
    float sumsq_b = 0.0f;
    uint32_t conv_count_start = 0;
    uint32_t tim_count_start = 0;

    void reset() {
        n = 0;
        sum_a = sumsq_a = 0.0f;
        sum_b = sumsq_b = 0.0f;
    }

    void accumulate(uint16_t rawA, uint16_t rawB) {
        n++;
        sum_a += rawA;
        sumsq_a += (float)rawA * rawA;
        sum_b += rawB;
        sumsq_b += (float)rawB * rawB;
    }
};

void streamPacket(
    uint8_t motor,
    uint16_t rawA,
    uint16_t rawB,
    float amps_a,
    float amps_b,
    float volts_a,
    float volts_b
) {
    const FOCStatusPacket packet = {
        FOC_STATUS_HEADER,
        motor,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        (float)rawA,
        (float)rawB,
        amps_a,
        amps_b,
        volts_a,
        volts_b,
    };
    Serial.write((uint8_t*)&packet, sizeof(packet));
}

// One USB write for both motors — avoids the second packet being dropped when the
// TX buffer fills between back-to-back Serial.write calls.
void streamDualPackets(
    uint16_t rawAx,
    uint16_t rawBx,
    float amps_ax,
    float amps_bx,
    float volts_ax,
    float volts_bx,
    uint16_t rawAy,
    uint16_t rawBy,
    float amps_ay,
    float amps_by,
    float volts_ay,
    float volts_by
) {
    alignas(4) uint8_t wire[2 * sizeof(FOCStatusPacket)];
    auto* packets = reinterpret_cast<FOCStatusPacket*>(wire);
    packets[0] = {
        FOC_STATUS_HEADER,
        1,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        (float)rawAx,
        (float)rawBx,
        amps_ax,
        amps_bx,
        volts_ax,
        volts_bx,
    };
    packets[1] = {
        FOC_STATUS_HEADER,
        2,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        (float)rawAy,
        (float)rawBy,
        amps_ay,
        amps_by,
        volts_ay,
        volts_by,
    };
    Serial.write(wire, sizeof(wire));
}

void printChannelSummary(
    const char* label,
    const ChannelStats& stats,
    InlineCurrentSenseSTM32G4& sense,
    uint32_t& conv_count_start,
    uint32_t& tim_count_start
) {
    if (stats.n == 0) return;

    const float mean_a = stats.sum_a / stats.n;
    const float mean_b = stats.sum_b / stats.n;
    const float std_a = sqrtf(max(0.0f, stats.sumsq_a / stats.n - mean_a * mean_a));
    const float std_b = sqrtf(max(0.0f, stats.sumsq_b / stats.n - mean_b * mean_b));
    const uint32_t conv_delta = sense.getAdcConvCount() - conv_count_start;
    conv_count_start = sense.getAdcConvCount();
    const uint32_t tim_delta = sense.getTimUpdateCount() - tim_count_start;
    tim_count_start = sense.getTimUpdateCount();

    Serial.print(label);
    Serial.print(": A ");
    Serial.print(mean_a, 1);
    Serial.print(" +/- ");
    Serial.print(std_a, 2);
    Serial.print(" LSB (");
    Serial.print(mean_a * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4);
    Serial.print(" V) | B ");
    Serial.print(mean_b, 1);
    Serial.print(" +/- ");
    Serial.print(std_b, 2);
    Serial.print(" LSB (");
    Serial.print(mean_b * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4);
    Serial.print(" V) | n=");
    Serial.print(stats.n);
    Serial.print(" | NDTR=");
    Serial.print(sense.getDmaNdtr());
    Serial.print(" OVR=");
    Serial.print(sense.isAdcOverrun() ? 1 : 0);
    Serial.print(" conv/s=");
    Serial.print(conv_delta);
    Serial.print(" tim/s=");
    Serial.println(tim_delta);
}

}  // namespace

inline void stm32g4_adc_dma_dual_test() {
    Serial.begin(BAUD_RATE);
    delay(2000);

    PWMDriverSTM32G4 pwmDriverX(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 pwmDriverY(N1_Y, N2_Y, N3_Y, EN_Y);
    InlineCurrentSenseSTM32G4 currentSenseX(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, pwmDriverX);
    InlineCurrentSenseSTM32G4 currentSenseY(ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, pwmDriverY);

    Serial.println("Stage 8.5: ADC TRGO+DMA dual test starting...");

    if (!pwmDriverX.init()) {
        Serial.println("Stage 8.5: PWM X init FAILED");
        while (true) delay(1000);
    }
    if (!pwmDriverY.init()) {
        Serial.println("Stage 8.5: PWM Y init FAILED");
        while (true) delay(1000);
    }
    pwmDriverX.enable();
    pwmDriverY.enable();
    pwmDriverX.setPwm(0.0f, 0.0f, 0.0f);
    pwmDriverY.setPwm(0.0f, 0.0f, 0.0f);

    // Phase-align TIM1 (X) and TIM8 (Y) so TRGO/ADC sampling is not skewed between motors.
    TIM_HandleTypeDef* syncMasters[2] = {
        pwmDriverX.getMasterTimerHandle(),
        pwmDriverY.getMasterTimerHandle(),
    };
    stm32_syncTimers(syncMasters, 2);

    Serial.println(
        String("Stage 8.5: analogRead pre-init X A0=") + analogRead(ADC_CS_X_1)
        + " A2=" + analogRead(ADC_CS_X_2)
        + " | Y A1=" + analogRead(ADC_CS_Y_1)
        + " A3=" + analogRead(ADC_CS_Y_2)
    );

    TIM_TypeDef* masterX = pwmDriverX.getMasterTimerInstance();
    TIM_TypeDef* masterY = pwmDriverY.getMasterTimerInstance();
    Serial.print("Stage 8.5: X master TIM");
    Serial.print(stm32_getTimerNumber(masterX));
    Serial.print(" RCR=");
    Serial.print(masterX != NULL && IS_TIM_REPETITION_COUNTER_INSTANCE(masterX)
        ? static_cast<int>(masterX->RCR) : -1);
    Serial.print(" | Y master TIM");
    Serial.print(stm32_getTimerNumber(masterY));
    Serial.print(" RCR=");
    Serial.println(masterY != NULL && IS_TIM_REPETITION_COUNTER_INSTANCE(masterY)
        ? static_cast<int>(masterY->RCR) : -1);

    currentSenseX.setAdcConvProbePin(TRGO_PROBE_X_PIN);
    currentSenseY.setAdcConvProbePin(TRGO_PROBE_Y_PIN);
    // Prepare Y (ADC1) before X (ADC2); DMA start order is Y-first inside commitDualDmaStart.
    if (!currentSenseY.init(true, true)) {
        Serial.println("Stage 8.5: current sense Y prepare FAILED");
        while (true) delay(1000);
    }
    if (!currentSenseX.init(true, true)) {
        Serial.println("Stage 8.5: current sense X prepare FAILED");
        while (true) delay(1000);
    }

    uint16_t pollY_a = 0, pollY_b = 0, pollX_a = 0, pollX_b = 0;
    const bool pollY_ok = currentSenseY.benchPollOnce(&pollY_a, &pollY_b);
    const bool pollX_ok = currentSenseX.benchPollOnce(&pollX_a, &pollX_b);
    Serial.print("Stage 8.5: bench poll pre-DMA Y ");
    Serial.print(pollY_ok ? "ok " : "FAIL ");
    Serial.print(pollY_a);
    Serial.print(",");
    Serial.print(pollY_b);
    Serial.print(" | X ");
    Serial.print(pollX_ok ? "ok " : "FAIL ");
    Serial.print(pollX_a);
    Serial.print(",");
    Serial.println(pollX_b);

    if (!InlineCurrentSenseSTM32G4::commitDualDmaStart(currentSenseX, currentSenseY)) {
        Serial.println("Stage 8.5: dual DMA start FAILED");
        while (true) delay(1000);
    }

    Serial.println("Stage 8.5: post-init DMA check:");
    currentSenseX.printHardwareDiagnostics();
    currentSenseY.printHardwareDiagnostics();
    if (!currentSenseX.validateDmaSampling(50) || !currentSenseY.validateDmaSampling(50)) {
        Serial.println("Stage 8.5: ERR: DMA sampling validation FAILED (buf stuck?)");
        while (true) delay(1000);
    }

    Serial.println("Stage 8.5: probe X=PA6 (D12), probe Y=PB5 (D4) toggle on ADC ConvCplt");
    Serial.println("Stage 8.5: streaming motor 1+2 at 200 Hz — foc_plotter @ 460800");
    Serial.println("Stage 8.5: switch foc_plotter to Motor 2 for Y traces (M1/M2 pkt rates shown in UI)");

    const unsigned long testStartMs = millis();
    TestPhase activePhase = TestPhase::Bias;
    Serial.println(String("Stage 8.5 phase: ") + phaseName(activePhase));

    ChannelStats statsX;
    ChannelStats statsY;
    statsX.conv_count_start = currentSenseX.getAdcConvCount();
    statsX.tim_count_start = currentSenseX.getTimUpdateCount();
    statsY.conv_count_start = currentSenseY.getAdcConvCount();
    statsY.tim_count_start = currentSenseY.getTimUpdateCount();

    unsigned long lastPacketTime = millis();
    unsigned long lastSummaryTime = millis();
    unsigned long lastPhaseApplyMs = millis();

    while (true) {
        const unsigned long elapsed = millis() - testStartMs;
        const TestPhase phase = phaseForElapsed(elapsed);
        if (phase != activePhase) {
            activePhase = phase;
            Serial.println(String("Stage 8.5 phase: ") + phaseName(activePhase));
        }

        if (millis() - lastPhaseApplyMs >= 10) {
            lastPhaseApplyMs = millis();
            applyPhase(pwmDriverX, pwmDriverY, activePhase, elapsed);
        }

        if (millis() - lastPacketTime >= FOC_STATUS_PERIOD_MS) {
            lastPacketTime = millis();

            uint16_t rawAx, rawBx;
            uint16_t rawAy, rawBy;
            if (!currentSenseX.readRawCodes(rawAx, rawBx)) {
                Serial.println("Stage 8.5: ADC X read failed");
                continue;
            }
            if (!currentSenseY.readRawCodes(rawAy, rawBy)) {
                Serial.println("Stage 8.5: ADC Y read failed");
                continue;
            }

            const float volts_ax = rawAx * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float volts_bx = rawBx * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float amps_ax = (volts_ax - currentSenseX.offset_ia) * currentSenseX.gain_a;
            const float amps_bx = (volts_bx - currentSenseX.offset_ib) * currentSenseX.gain_b;
            statsX.accumulate(rawAx, rawBx);

            const float volts_ay = rawAy * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float volts_by = rawBy * InlineCurrentSenseSTM32G4::adc_voltage_conv;
            const float amps_ay = (volts_ay - currentSenseY.offset_ia) * currentSenseY.gain_a;
            const float amps_by = (volts_by - currentSenseY.offset_ib) * currentSenseY.gain_b;
            streamDualPackets(
                rawAx, rawBx, amps_ax, amps_bx, volts_ax, volts_bx,
                rawAy, rawBy, amps_ay, amps_by, volts_ay, volts_by
            );
            statsY.accumulate(rawAy, rawBy);
        }

        if (millis() - lastSummaryTime >= 1000 && statsX.n > 0 && statsY.n > 0) {
            lastSummaryTime = millis();
            printChannelSummary("X", statsX, currentSenseX, statsX.conv_count_start, statsX.tim_count_start);
            printChannelSummary("Y", statsY, currentSenseY, statsY.conv_count_start, statsY.tim_count_start);
            statsX.reset();
            statsY.reset();
        }
    }
}

#endif  // STM32G4xx
