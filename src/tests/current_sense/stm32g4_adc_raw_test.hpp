// Stage 7 bench test — ADC raw, software start (docs/STM32G4_upgrade_stages.md).

//

// Polls both current-sense channels at 200 Hz (software-triggered, no DMA,

// no TRGO; PWM driver constructed but never init'ed, so outputs stay off)

// and streams FOCStatusPackets for UI/foc_plotter.py at BAUD_RATE 460800.

//

// Packet fields (FOCStatusPacket — see Interface.hpp):
//   raw_adc_a, raw_adc_b  <- raw 12-bit codes [LSB]
//   amps_a, amps_b        <- nominal phase currents [A]
//   volts_a, volts_b      <- pin voltages [V]
//   all motion / Iq fields <- 0

//

// A 1 Hz text summary (mean / std per channel in LSB, mean volts) lands in

// the plotter's log panel; the bias and noise pass criteria read from it.

#pragma once



#include "Arduino.h"



#if defined(STM32G4xx)



#include "interface/Interface.hpp" // FOCStatusPacket, FOC_STATUS_HEADER

#include "current_sense/stm32g4/InlineCurrentSenseSTM32G4.hpp"

#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"

#include "utils/constants.h"

#include "utils/tweakable_params.h"



inline void stm32g4_adc_raw_test() {

    Serial.begin(460800);

    delay(2000);



    // Constructed for the final class signature only — init() is never

    // called, so the PWM outputs stay off for the whole test (Stage 7 rig).

    PWMDriverSTM32G4 pwmDriverX(N1_X, N2_X, N3_X, EN_X);

    InlineCurrentSenseSTM32G4 currentSense(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, pwmDriverX);



    // analogRead before we claim ADC2 — expect ~2048 LSB (3.3 V VDD pad) or

    // ~3100 LSB if the shield presents ACS712 output (~2.5 V) at the pin.

    Serial.println(

        "analogRead sanity (pre-init): A0=" + String(analogRead(ADC_CS_X_1))

        + " A2=" + String(analogRead(ADC_CS_X_2))

    );



    Serial.println("Stage 7: ADC raw test (software start, polled, PWM off)");

    if (!currentSense.init(false)) {

        Serial.println("Stage 7: current sense init FAILED");

        while (true) delay(1000);

    }

    Serial.println("Stage 7: current sense initialised, streaming at 200 Hz");



    // 1 s summary accumulators (LSB domain)

    uint32_t n = 0;

    float sum_a = 0.0f, sumsq_a = 0.0f;

    float sum_b = 0.0f, sumsq_b = 0.0f;



    unsigned long lastPacketTime = millis();

    unsigned long lastSummaryTime = millis();



    while (true) {

        if (millis() - lastPacketTime >= FOC_STATUS_PERIOD_MS) {

            lastPacketTime = millis();



            uint16_t rawA, rawB;

            if (!currentSense.readRawCodes(rawA, rawB)) {

                Serial.println("Stage 7: ADC poll timeout");

                continue;

            }



            float volts_a = rawA * InlineCurrentSenseSTM32G4::adc_voltage_conv;

            float volts_b = rawB * InlineCurrentSenseSTM32G4::adc_voltage_conv;

            float amps_a = (volts_a - currentSense.offset_ia) * currentSense.gain_a;

            float amps_b = (volts_b - currentSense.offset_ib) * currentSense.gain_b;



            FOCStatusPacket packet = {
                FOC_STATUS_HEADER,
                1,
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



            n++;

            sum_a += rawA; sumsq_a += (float)rawA * rawA;

            sum_b += rawB; sumsq_b += (float)rawB * rawB;

        }



        if (millis() - lastSummaryTime >= 1000 && n > 0) {

            lastSummaryTime = millis();



            float mean_a = sum_a / n;

            float mean_b = sum_b / n;

            float std_a = sqrtf(max(0.0f, sumsq_a / n - mean_a * mean_a));

            float std_b = sqrtf(max(0.0f, sumsq_b / n - mean_b * mean_b));



            Serial.println(

                "A: " + String(mean_a, 1) + " +/- " + String(std_a, 2) + " LSB ("

                + String(mean_a * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4) + " V) | B: "

                + String(mean_b, 1) + " +/- " + String(std_b, 2) + " LSB ("

                + String(mean_b * InlineCurrentSenseSTM32G4::adc_voltage_conv, 4) + " V) | n="

                + String(n)

            );



            n = 0;

            sum_a = sumsq_a = 0.0f;

            sum_b = sumsq_b = 0.0f;

        }

    }

}



#endif // STM32G4xx


