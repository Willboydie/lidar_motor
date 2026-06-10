// Inline current sense for STM32G4 + SimpleFOC Shield 3.2 (ACS712).
// Target architecture: docs/STM32G4_upgrade_plan copy.md §3.
//
// Standalone class — does NOT inherit from imported SimpleFOC CurrentSense.
#pragma once

#include "Arduino.h"

#if defined(STM32G4xx)

#include "stm32g4xx_hal.h"

struct PhaseCurrent_s {
    float a;
    float b;
    float c;
};

struct ABCurrent_s {
    float alpha;
    float beta;
};

struct DQCurrent_s {
    float d;
    float q;
};

class PWMDriverSTM32G4;

class InlineCurrentSenseSTM32G4 {
public:
    InlineCurrentSenseSTM32G4(float mVpA, int pinA, int pinB, PWMDriverSTM32G4& driver);

    bool init(bool hardware_trigger = true); // false = Stage 7 software scan
    void calibrateOffsets(int rounds = 1000);
    PhaseCurrent_s getPhaseCurrents(); // non-blocking buffer read (§3.4)
    DQCurrent_s getFOCCurrents(float angle_el);

    // Stage 7 bench helper: refresh software buffer or read latest DMA sample.
    bool readRawCodes(uint16_t& rawA, uint16_t& rawB);

    ADC_TypeDef* getADCInstance() const { return adc_handle.Instance; }
    bool isHardwareTriggered() const { return hardware_dma_; }

    float gain_a;
    float gain_b;
    float gain_c;
    float offset_ia;
    float offset_ib;
    float offset_ic;

    static constexpr float adc_voltage_conv = 3.3f / 4096.0f;

private:
    bool _pollSampleBuf();

    int pinA;
    int pinB;
    PWMDriverSTM32G4& driver;
    ADC_HandleTypeDef adc_handle = {};
    DMA_HandleTypeDef hdma = {};
    volatile uint16_t sample_buf[2] = {0, 0};
    bool hardware_dma_ = false;
    bool initialised = false;
};

#endif // STM32G4xx
