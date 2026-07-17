// Inline current sense for STM32G4 + SimpleFOC Shield 3.2 (ACS712).
// Target architecture: docs/STM32G4_upgrade_plan copy.md §3.
//
// Hardware backend of InlineCurrentSense: TRGO + DMA ADC sampling on the G4.
#pragma once

#include "Arduino.h"
#include "../InlineCurrentSense.hpp"
#include "../../fault_manager/FaultManager.hpp"
#include "../../utils/constants.h"

#if defined(STM32G4xx)

#include "stm32g4xx_hal.h"

class PWMDriverSTM32G4;

class InlineCurrentSenseSTM32G4 : public InlineCurrentSense {
public:
    InlineCurrentSenseSTM32G4(float mVpA, int pinA, int pinB, PWMDriverSTM32G4& driver);

    // hardware_trigger=true: §3.2 TRGO+DMA (production). false: Stage 7 software scan only.
    // defer_dma_start=true: run steps 1–7 only; call startDma() or commitDualDmaStart() next.
    bool init(bool hardware_trigger = true, bool defer_dma_start = false) override;
    bool startDma();
    static bool commitDualDmaStart(InlineCurrentSenseSTM32G4& a, InlineCurrentSenseSTM32G4& b);

    void calibrateOffsets(int rounds = 1000);
    PhaseCurrent_s getPhaseCurrents() override; // non-blocking buffer read (§3.4)

    // Stage 7 bench helper: refresh software buffer or read latest DMA sample.
    bool readRawCodes(uint16_t& rawA, uint16_t& rawB);

    ADC_TypeDef* getADCInstance() const { return adc_handle.Instance; }
    bool isHardwareTriggered() const { return hardware_dma_; }
    bool isDmaRunning() const { return dma_running_; }
    uint16_t getDmaNdtr() const;
    bool isAdcOverrun() const;
    uint32_t getAdcConvCount() const;
    uint32_t getTimUpdateCount() const;

    // Post-start health check: conv count must move and samples must look like real bias.
    bool validateDmaSampling(uint32_t settle_ms = 50) const;

    // Stage 8.5 bench: one triggered scan before DMA (defer_dma_start path only).
    bool benchPollOnce(uint16_t* rawA = nullptr, uint16_t* rawB = nullptr);

    // Stage 8 scope probe: toggles on each DMA sequence complete (pass -1 to disable).
    void setAdcConvProbePin(int pin);

    // One-line hardware health snapshot for the Stage 8 log panel.
    void printHardwareDiagnostics() const;

    // Called from HAL_ADC_ConvCpltCallback (same translation unit).
    void notifyConvComplete();

    FaultCode csInitFault() const {
        return (pinA == ADC_CS_Y_1 || pinA == ADC_CS_Y_2) ? FAULT_CS_Y_INIT : FAULT_CS_X_INIT;
    }

    static constexpr float adc_voltage_conv = 3.3f / 4096.0f;

private:
    bool _pollSampleBuf();
    bool _prepareHardware(PinName pn_a, PinName pn_b, ADC_TypeDef* adc, TIM_HandleTypeDef* master, uint32_t trgo);
    bool _initSoftware(PinName pn_a, PinName pn_b, ADC_TypeDef* adc);
    void _stopDmaConversions();
    void _reapplyHardwareTrigger();
    static void _quiesceAllAdc12();
    static void _tearDownDma(InlineCurrentSenseSTM32G4* owner, ADC_HandleTypeDef& hadc, DMA_HandleTypeDef& hdma);

    int pinA;
    int pinB;
    PWMDriverSTM32G4& driver;
    ADC_HandleTypeDef adc_handle = {};
    DMA_HandleTypeDef hdma = {};
    alignas(4) volatile uint16_t sample_buf[2] = {0, 0};
    TIM_TypeDef* master_tim_ = NULL;
    bool hardware_dma_ = false;
    bool dma_running_ = false;
    bool initialised = false;
    int probe_pin_ = -1;
    volatile uint32_t adc_conv_count_ = 0;
    volatile uint32_t tim_update_count_ = 0;
};

#endif // STM32G4xx
