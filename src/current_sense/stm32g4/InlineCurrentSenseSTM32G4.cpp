#include "InlineCurrentSenseSTM32G4.hpp"
#if defined(STM32G4xx)
#include "PeripheralPins.h"
#include "pinmap.h"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "pwm_driver/stm32g4/stm32_mcu.h"
#include "pwm_driver/stm32g4/stm32_timerutils.h"
#include "utils/quick_defs.h"
#include "utils/constants.h"
#include "fault_manager/FaultManager.hpp"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_tim.h"

static constexpr uint32_t kAdcPollTimeoutMs = 100;
// upgrade plan §3.2 step 4
static constexpr uint32_t kAdcSampleTime = ADC_SAMPLETIME_2CYCLES_5;

static InlineCurrentSenseSTM32G4* g_adc1_dma_owner = NULL;
static InlineCurrentSenseSTM32G4* g_adc2_dma_owner = NULL;
static InlineCurrentSenseSTM32G4* g_adc1_hw_prepared = NULL;
static InlineCurrentSenseSTM32G4* g_adc2_hw_prepared = NULL;
static DMA_HandleTypeDef* g_dma1_ch1_hdma = NULL;
static DMA_HandleTypeDef* g_dma1_ch2_hdma = NULL;

// ---------- PinMap / ADC helpers (upgrade plan §3.2 steps 1–2) ----------
static uint32_t _adcChannelForPin(PinName pin, ADC_TypeDef* adc) {
    for (int i = 0; PinMap_ADC[i].pin != NC; i++) {
        if (pin != (PinMap_ADC[i].pin & ~ALTX_MASK)) continue;
        if ((ADC_TypeDef*)PinMap_ADC[i].peripheral != adc) continue;
        return __HAL_ADC_DECIMAL_NB_TO_CHANNEL(STM_PIN_CHANNEL(PinMap_ADC[i].function));
    }
    return 0xFFFFFFFF;
}
static ADC_TypeDef* _findCommonADC(PinName pinA, PinName pinB) {
    for (int i = 0; PinMap_ADC[i].pin != NC; i++) {
        if (pinA != (PinMap_ADC[i].pin & ~ALTX_MASK)) continue;
        ADC_TypeDef* adc = (ADC_TypeDef*)PinMap_ADC[i].peripheral;
        if (_adcChannelForPin(pinB, adc) != 0xFFFFFFFF) return adc;
    }
    return nullptr;
}
static bool _adcPinInitForAdc(PinName pin, ADC_TypeDef* adc) {
    for (int i = 0; PinMap_ADC[i].pin != NC; i++) {
        if (pin != (PinMap_ADC[i].pin & ~ALTX_MASK)) continue;
        if ((ADC_TypeDef*)PinMap_ADC[i].peripheral != adc) continue;
        pinmap_pinout(PinMap_ADC[i].pin, PinMap_ADC);
        return true;
    }
    return false;
}
static void _pauseMasterTim(TIM_TypeDef* tim) {
    if (tim != NULL) LL_TIM_DisableCounter(tim);
}
static void _resumeMasterTim(TIM_TypeDef* tim) {
    if (tim != NULL) LL_TIM_EnableCounter(tim);
}
static void _adcClockEnable(ADC_TypeDef* adc) {
    if (adc == ADC1 || adc == ADC2) {
#ifdef __HAL_RCC_ADC12_CLK_ENABLE
        __HAL_RCC_ADC12_CLK_ENABLE();
#endif
    }
#ifdef ADC3
    else if (adc == ADC3) {
#if defined(ADC345_COMMON)
        __HAL_RCC_ADC345_CLK_ENABLE();
#endif
    }
#endif
#ifdef ADC4
    else if (adc == ADC4) {
#if defined(ADC345_COMMON)
        __HAL_RCC_ADC345_CLK_ENABLE();
#endif
    }
#endif
#ifdef ADC5
    else if (adc == ADC5) {
#if defined(ADC345_COMMON)
        __HAL_RCC_ADC345_CLK_ENABLE();
#endif
    }
#endif
}
static InlineCurrentSenseSTM32G4* _ownerForAdc(ADC_TypeDef* adc) {
    if (adc == ADC1) return g_adc1_dma_owner;
    if (adc == ADC2) return g_adc2_dma_owner;
    return NULL;
}
static void _ensureAdcIndependent(ADC_TypeDef* adc) {
#if defined(ADC12_COMMON)
    if (adc == ADC1 || adc == ADC2) {
        LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_INDEPENDENT);
    }
#endif
}
static void _registerDmaOwner(InlineCurrentSenseSTM32G4* owner, DMA_HandleTypeDef* hdma) {
    if (owner == NULL || hdma == NULL || hdma->Instance == NULL) return;
    if (hdma->Instance == DMA1_Channel1) {
        g_dma1_ch1_hdma = hdma;
        g_adc1_dma_owner = owner;
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    } else if (hdma->Instance == DMA1_Channel2) {
        g_dma1_ch2_hdma = hdma;
        g_adc2_dma_owner = owner;
        HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    }
}
static void _unregisterDmaOwner(InlineCurrentSenseSTM32G4* owner, DMA_HandleTypeDef* hdma) {
    if (hdma == NULL || hdma->Instance == NULL) return;
    if (hdma->Instance == DMA1_Channel1) {
        if (g_adc1_dma_owner == owner) g_adc1_dma_owner = NULL;
        if (g_dma1_ch1_hdma == hdma) g_dma1_ch1_hdma = NULL;
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    } else if (hdma->Instance == DMA1_Channel2) {
        if (g_adc2_dma_owner == owner) g_adc2_dma_owner = NULL;
        if (g_dma1_ch2_hdma == hdma) g_dma1_ch2_hdma = NULL;
        HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    }
}
static void _applyRegularTriggerToLL(ADC_TypeDef* adc, TIM_TypeDef* timer) {
    if (adc == NULL || timer == NULL) return;
    if (timer == TIM1) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
    } else if (timer == TIM8) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM8_TRGO);
    } else if (timer == TIM2) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
    } else if (timer == TIM3) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
    } else if (timer == TIM4) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM4_TRGO);
    } else if (timer == TIM6) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM6_TRGO);
    } else if (timer == TIM7) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM7_TRGO);
    } else if (timer == TIM15) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM15_TRGO);
    } else if (timer == TIM20) {
        LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM20_TRGO);
    } else {
        return;
    }
    LL_ADC_REG_SetTriggerEdge(adc, LL_ADC_REG_TRIG_EXT_RISING);
}
static void _ensureUnlimitedDma(ADC_TypeDef* adc) {
    LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
}
static void _disableAdcIfEnabled(ADC_TypeDef* adc) {
    if (adc == NULL || !LL_ADC_IsEnabled(adc)) return;
    if (LL_ADC_REG_IsConversionOngoing(adc)) {
        LL_ADC_REG_StopConversion(adc);
    }
    LL_ADC_Disable(adc);
}
static uint32_t _timerToRegularTRGO(TIM_TypeDef* timer) {
    if (timer == TIM1) return ADC_EXTERNALTRIG_T1_TRGO;
#ifdef TIM2
    if (timer == TIM2) return ADC_EXTERNALTRIG_T2_TRGO;
#endif
#ifdef TIM3
    if (timer == TIM3) return ADC_EXTERNALTRIG_T3_TRGO;
#endif
#ifdef TIM4
    if (timer == TIM4) return ADC_EXTERNALTRIG_T4_TRGO;
#endif
#ifdef TIM6
    if (timer == TIM6) return ADC_EXTERNALTRIG_T6_TRGO;
#endif
#ifdef TIM7
    if (timer == TIM7) return ADC_EXTERNALTRIG_T7_TRGO;
#endif
#ifdef TIM8
    if (timer == TIM8) return ADC_EXTERNALTRIG_T8_TRGO;
#endif
#ifdef TIM15
    if (timer == TIM15) return ADC_EXTERNALTRIG_T15_TRGO;
#endif
#ifdef TIM20
    if (timer == TIM20) return ADC_EXTERNALTRIG_T20_TRGO;
#endif
    return 0xFFFFFFFF;
}
static void _resetAdcHandle(ADC_HandleTypeDef& hadc, ADC_TypeDef* adc) {
    HAL_ADC_DeInit(&hadc);
    hadc = {};
    hadc.Instance = adc;
    hadc.State = HAL_ADC_STATE_RESET;
    hadc.Lock = HAL_UNLOCKED;
    hadc.DMA_Handle = NULL;
}
void InlineCurrentSenseSTM32G4::_tearDownDma(
    InlineCurrentSenseSTM32G4* owner,
    ADC_HandleTypeDef& hadc,
    DMA_HandleTypeDef& hdma
) {
    if (owner != NULL) owner->_stopDmaConversions();
    _unregisterDmaOwner(owner, &hdma);
    if (hadc.Instance != NULL) {
        HAL_ADC_Stop_DMA(&hadc);
        HAL_ADC_DeInit(&hadc);
    }
    if (hdma.Instance != NULL) {
        HAL_DMA_DeInit(&hdma);
    }
    hdma = {};
    hadc.DMA_Handle = NULL;
}
static bool _configureDmaForAdc(ADC_HandleTypeDef& hadc, DMA_HandleTypeDef& hdma) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    if (hadc.Instance == ADC1) {
        hdma.Instance = DMA1_Channel1;
        hdma.Init.Request = DMA_REQUEST_ADC1;
    } else if (hadc.Instance == ADC2) {
        hdma.Instance = DMA1_Channel2;
        hdma.Init.Request = DMA_REQUEST_ADC2;
    } else {
        return false;
    }
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma) != HAL_OK) return false;
    hadc.DMA_Handle = &hdma;
    __HAL_LINKDMA(&hadc, DMA_Handle, hdma);
    return true;
}
static void _applySoftwareAdcFields(ADC_HandleTypeDef& hadc) {
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
}
static void _applyHardwareAdcFields(ADC_HandleTypeDef& hadc, uint32_t trgo) {
    hadc.Init.ExternalTrigConv = trgo;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
}
static bool _configureAdcChannels(
    ADC_HandleTypeDef& hadc,
    PinName pn_a,
    PinName pn_b,
    ADC_TypeDef* adc,
    FaultCode fault
) {
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.SamplingTime = kAdcSampleTime;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.Channel = _adcChannelForPin(pn_a, adc);
    if (sConfig.Channel == 0xFFFFFFFF || HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        faultManager.addFault(fault);
        return false;
    }
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.Channel = _adcChannelForPin(pn_b, adc);
    if (sConfig.Channel == 0xFFFFFFFF || HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        faultManager.addFault(fault);
        return false;
    }
    return true;
}
static void _logHardwareConfig(
    int pin_a,
    int pin_b,
    ADC_TypeDef* adc,
    TIM_HandleTypeDef* master,
    PinName pn_a,
    PinName pn_b
) {
    Serial.print("CS: TRGO+DMA on ADC");
    Serial.print((adc == ADC1) ? 1 : (adc == ADC2) ? 2 : 0);
    Serial.print(", master TIM");
    Serial.print(stm32_getTimerNumber(master->Instance));
    Serial.print(", pins A");
    Serial.print(pin_a);
    Serial.print("/B");
    Serial.print(pin_b);
    Serial.print(", rank1=IN");
    Serial.print((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_a, adc)));
    Serial.print(" rank2=IN");
    Serial.println((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_b, adc)));
}

// ---------- InlineCurrentSenseSTM32G4 ----------
InlineCurrentSenseSTM32G4::InlineCurrentSenseSTM32G4(float mVpA, int pinA_, int pinB_, PWMDriverSTM32G4& _driver)
: pinA(pinA_)
, pinB(pinB_)
, driver(_driver)
{
    gain_a = gain_b = gain_c = 1000.0f / mVpA;
    offset_ia = offset_ib = 1.65f;  // ACS712 mid-rail bias, refined by calibrateOffsets()
    offset_ic = 0.0f;
}
void InlineCurrentSenseSTM32G4::_quiesceAllAdc12() {
    if (g_adc1_dma_owner != NULL) g_adc1_dma_owner->_stopDmaConversions();
    if (g_adc2_dma_owner != NULL) g_adc2_dma_owner->_stopDmaConversions();
}
void InlineCurrentSenseSTM32G4::_reapplyHardwareTrigger() {
    if (!hardware_dma_ || adc_handle.Instance == NULL || master_tim_ == NULL) return;
    if (!LL_ADC_IsEnabled(adc_handle.Instance)) {
        LL_ADC_Enable(adc_handle.Instance);
    }
    _applyRegularTriggerToLL(adc_handle.Instance, master_tim_);
    _ensureUnlimitedDma(adc_handle.Instance);
}
bool InlineCurrentSenseSTM32G4::_pollSampleBuf() {
    HAL_ADC_Stop(&adc_handle);
    if (HAL_ADC_Start(&adc_handle) != HAL_OK) {
        return false;
    }
    if (HAL_ADC_PollForConversion(&adc_handle, kAdcPollTimeoutMs) != HAL_OK) {
        HAL_ADC_Stop(&adc_handle);
        return false;
    }
    sample_buf[0] = (uint16_t)HAL_ADC_GetValue(&adc_handle);
    if (HAL_ADC_PollForConversion(&adc_handle, kAdcPollTimeoutMs) != HAL_OK) {
        HAL_ADC_Stop(&adc_handle);
        return false;
    }
    sample_buf[1] = (uint16_t)HAL_ADC_GetValue(&adc_handle);
    HAL_ADC_Stop(&adc_handle);
    return true;
}
bool InlineCurrentSenseSTM32G4::_prepareHardware(
    PinName pn_a,
    PinName pn_b,
    ADC_TypeDef* adc,
    TIM_HandleTypeDef* master,
    uint32_t trgo
) {
    // §3.2 steps 3–7: ADC init, channels, DMA link, calibrate, map ADC trigger.
    // TRGO+RCR on the master timer is already set in configure3PWM (§2.2) — do not pause/reconfigure it here.
    _tearDownDma(this, adc_handle, hdma);
    _resetAdcHandle(adc_handle, adc);
    adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc_handle.Init.Resolution = ADC_RESOLUTION_12B;
    adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc_handle.Init.ContinuousConvMode = DISABLE;
    adc_handle.Init.LowPowerAutoWait = DISABLE;
    adc_handle.Init.GainCompensation = 0;
    adc_handle.Init.DiscontinuousConvMode = DISABLE;
    adc_handle.Init.NbrOfDiscConversion = 1;
    adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_handle.Init.NbrOfConversion = 2;
    adc_handle.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    adc_handle.Init.OversamplingMode = DISABLE;
    _applyHardwareAdcFields(adc_handle, trgo);
    if (HAL_ADC_Init(&adc_handle) != HAL_OK) {
        faultManager.addFault(csInitFault());
        return false;
    }
    if (!_configureAdcChannels(adc_handle, pn_a, pn_b, adc, csInitFault())) {
        return false;
    }
    if (!_configureDmaForAdc(adc_handle, hdma)) {
        faultManager.addFault(csInitFault());
        return false;
    }
    // RM: do not calibrate ADC1 while ADC2 is converting (and vice versa).
    ADC_TypeDef* sibling = (adc == ADC1) ? ADC2 : ADC1;
    InlineCurrentSenseSTM32G4* sibling_prepared =
        (adc == ADC1) ? g_adc2_hw_prepared : g_adc1_hw_prepared;
    _quiesceAllAdc12();
    _disableAdcIfEnabled(sibling);
    _pauseMasterTim(master->Instance);
    const bool cal_ok = (HAL_ADCEx_Calibration_Start(&adc_handle, ADC_SINGLE_ENDED) == HAL_OK);
    _resumeMasterTim(master->Instance);
    if (!cal_ok) {
        faultManager.addFault(csInitFault());
        return false;
    }
    master_tim_ = master->Instance;
    _applyRegularTriggerToLL(adc_handle.Instance, master->Instance);
    _ensureUnlimitedDma(adc_handle.Instance);
    if (adc == ADC1) {
        g_adc1_hw_prepared = this;
    } else if (adc == ADC2) {
        g_adc2_hw_prepared = this;
    }
    if (sibling_prepared != NULL) {
        sibling_prepared->_reapplyHardwareTrigger();
    }
    _logHardwareConfig(pinA, pinB, adc, master, pn_a, pn_b);
    return true;
}
bool InlineCurrentSenseSTM32G4::_initSoftware(PinName pn_a, PinName pn_b, ADC_TypeDef* adc) {
    _tearDownDma(this, adc_handle, hdma);
    _resetAdcHandle(adc_handle, adc);
    adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc_handle.Init.Resolution = ADC_RESOLUTION_12B;
    adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc_handle.Init.ContinuousConvMode = DISABLE;
    adc_handle.Init.LowPowerAutoWait = DISABLE;
    adc_handle.Init.GainCompensation = 0;
    adc_handle.Init.DiscontinuousConvMode = DISABLE;
    adc_handle.Init.NbrOfDiscConversion = 1;
    adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_handle.Init.NbrOfConversion = 2;
    adc_handle.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    adc_handle.Init.OversamplingMode = DISABLE;
    _applySoftwareAdcFields(adc_handle);
    if (HAL_ADC_Init(&adc_handle) != HAL_OK) {
        faultManager.addFault(csInitFault());
        return false;
    }
    if (!_configureAdcChannels(adc_handle, pn_a, pn_b, adc, csInitFault())) {
        return false;
    }
    if (HAL_ADCEx_Calibration_Start(&adc_handle, ADC_SINGLE_ENDED) != HAL_OK) {
        faultManager.addFault(csInitFault());
        return false;
    }
    Serial.print("CS: software scan on ADC");
    Serial.print((adc == ADC1) ? 1 : (adc == ADC2) ? 2 : 0);
    Serial.print(" rank1=IN");
    Serial.print((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_a, adc)));
    Serial.print(" rank2=IN");
    Serial.println((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_b, adc)));
    return true;
}
bool InlineCurrentSenseSTM32G4::startDma() {
    if (!hardware_dma_ || !initialised || dma_running_ || adc_handle.Instance == NULL) {
        return dma_running_;
    }
    if (master_tim_ == NULL) {
        faultManager.addFault(csInitFault());
        return false;
    }
    if ((master_tim_->CR2 & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_UPDATE) {
        faultManager.addFault(csInitFault());
        return false;
    }
    PinName pn_a = analogInputToPinName(pinA);
    PinName pn_b = analogInputToPinName(pinB);
    if (pn_a != NC && pn_b != NC && adc_handle.Instance != NULL) {
        if (!_adcPinInitForAdc(pn_a, adc_handle.Instance) || !_adcPinInitForAdc(pn_b, adc_handle.Instance)) {
            faultManager.addFault(csInitFault());
            return false;
        }
    }
    sample_buf[0] = 0;
    sample_buf[1] = 0;
    adc_conv_count_ = 0;
    tim_update_count_ = 0;
    _registerDmaOwner(this, &hdma);
    if (HAL_ADC_Start_DMA(&adc_handle, (uint32_t*)sample_buf, 2) != HAL_OK) {
        faultManager.addFault(csInitFault());
        _unregisterDmaOwner(this, &hdma);
        return false;
    }
    _ensureUnlimitedDma(adc_handle.Instance);
    __HAL_ADC_CLEAR_FLAG(&adc_handle, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    dma_running_ = true;
    return true;
}
bool InlineCurrentSenseSTM32G4::commitDualDmaStart(
    InlineCurrentSenseSTM32G4& a,
    InlineCurrentSenseSTM32G4& b
) {
    // Start the second argument first so ADC1 (typically motor Y) claims DMA1 Ch1
    // before ADC2 starts on Ch2 — avoids Ch1 init while Ch2 is already streaming.
    if (!b.startDma()) {
        faultManager.addFault(b.csInitFault());
        return false;
    }
    if (!a.startDma()) {
        faultManager.addFault(a.csInitFault());
        b._stopDmaConversions();
        return false;
    }
    a._reapplyHardwareTrigger();
    b._reapplyHardwareTrigger();
    return true;
}
void InlineCurrentSenseSTM32G4::_stopDmaConversions() {
    if (!dma_running_ || adc_handle.Instance == NULL) return;
    HAL_ADC_Stop_DMA(&adc_handle);
    _unregisterDmaOwner(this, &hdma);
    dma_running_ = false;
}
bool InlineCurrentSenseSTM32G4::init(bool hardware_trigger, bool defer_dma_start) {
    PinName pn_a = analogInputToPinName(pinA);
    PinName pn_b = analogInputToPinName(pinB);
    if (pn_a == NC || pn_b == NC) {
        faultManager.addFault(csInitFault());
        return false;
    }
    ADC_TypeDef* adc = _findCommonADC(pn_a, pn_b);
    if (adc == nullptr) {
        faultManager.addFault(csInitFault());
        return false;
    }
    _quiesceAllAdc12();
    _adcClockEnable(adc);
    _ensureAdcIndependent(adc);
    if (!_adcPinInitForAdc(pn_a, adc) || !_adcPinInitForAdc(pn_b, adc)) {
        faultManager.addFault(csInitFault());
        return false;
    }

    if (!hardware_trigger) {
        hardware_dma_ = false;
        master_tim_ = NULL;
        dma_running_ = false;
        if (!_initSoftware(pn_a, pn_b, adc)) {
            return false;
        }
        initialised = true;
        return true;
    }

    TIM_HandleTypeDef* master = driver.getMasterTimerHandle();
    if (master == NULL) {
        faultManager.addFault(csInitFault());
        return false;
    }
    const uint32_t trgo = _timerToRegularTRGO(master->Instance);
    if (trgo == 0xFFFFFFFF) {
        faultManager.addFault(csInitFault());
        return false;
    }
    hardware_dma_ = true;
    dma_running_ = false;
    if (!_prepareHardware(pn_a, pn_b, adc, master, trgo)) {
        hardware_dma_ = false;
        return false;
    }
    initialised = true;
    if (defer_dma_start) {
        return true;
    }
    if (!startDma()) {
        faultManager.addFault(csInitFault());
        initialised = false;
        hardware_dma_ = false;
        _tearDownDma(this, adc_handle, hdma);
        return false;
    }
    printHardwareDiagnostics();
    return true;
}
bool InlineCurrentSenseSTM32G4::validateDmaSampling(uint32_t settle_ms) const {
    if (!hardware_dma_ || !dma_running_) return false;
    const uint32_t c0 = adc_conv_count_;
    delay(settle_ms);
    if (adc_conv_count_ <= c0) return false;
    return (sample_buf[0] > 500U || sample_buf[1] > 500U);
}
bool InlineCurrentSenseSTM32G4::benchPollOnce(uint16_t* rawA, uint16_t* rawB) {
    if (!initialised || dma_running_) return false;
    if (!_pollSampleBuf()) return false;
    if (rawA != nullptr) *rawA = sample_buf[0];
    if (rawB != nullptr) *rawB = sample_buf[1];
    return true;
}
void InlineCurrentSenseSTM32G4::calibrateOffsets(int rounds) {
    double sum_a = 0.0;
    double sum_b = 0.0;
    for (int i = 0; i < rounds; ++i) {
        if (!hardware_dma_) _pollSampleBuf();
        sum_a += sample_buf[0] * adc_voltage_conv;
        sum_b += sample_buf[1] * adc_voltage_conv;
        delayMicroseconds(50);
    }
    offset_ia = (float)(sum_a / rounds);
    offset_ib = (float)(sum_b / rounds);
    offset_ic = 0.0f;
}
PhaseCurrent_s InlineCurrentSenseSTM32G4::getPhaseCurrents() {
    float Va = sample_buf[0] * adc_voltage_conv;
    float Vb = sample_buf[1] * adc_voltage_conv;
    PhaseCurrent_s i;
    i.a = (Va - offset_ia) * gain_a;
    i.b = (Vb - offset_ib) * gain_b;
    i.c = -(i.a + i.b);
    return i;
}
bool InlineCurrentSenseSTM32G4::readRawCodes(uint16_t& rawA, uint16_t& rawB) {
    if (!initialised) return false;
    if (hardware_dma_ && master_tim_ != NULL) {
        if (LL_TIM_IsActiveFlag_UPDATE(master_tim_)) {
            LL_TIM_ClearFlag_UPDATE(master_tim_);
            tim_update_count_++;
        }
    }
    if (!hardware_dma_ && !_pollSampleBuf()) return false;
    rawA = sample_buf[0];
    rawB = sample_buf[1];
    return true;
}
void InlineCurrentSenseSTM32G4::setAdcConvProbePin(int pin) {
    probe_pin_ = pin;
    if (pin >= 0) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}
void InlineCurrentSenseSTM32G4::notifyConvComplete() {
    adc_conv_count_++;
    if (probe_pin_ >= 0) {
        digitalWrite(probe_pin_, digitalRead(probe_pin_) ? LOW : HIGH);
    }
}
uint16_t InlineCurrentSenseSTM32G4::getDmaNdtr() const {
    if (!hardware_dma_ || hdma.Instance == NULL) return 0;
    return __HAL_DMA_GET_COUNTER(&hdma);
}
bool InlineCurrentSenseSTM32G4::isAdcOverrun() const {
    if (adc_handle.Instance == NULL) return false;
    return __HAL_ADC_GET_FLAG(&adc_handle, ADC_FLAG_OVR) != RESET;
}
uint32_t InlineCurrentSenseSTM32G4::getAdcConvCount() const {
    return adc_conv_count_;
}
uint32_t InlineCurrentSenseSTM32G4::getTimUpdateCount() const {
    return tim_update_count_;
}
void InlineCurrentSenseSTM32G4::printHardwareDiagnostics() const {
    if (!hardware_dma_ || adc_handle.Instance == NULL) return;
    const uint32_t cfgr = adc_handle.Instance->CFGR;
    const uint32_t cr = adc_handle.Instance->CR;
    const uint32_t cr1 = (master_tim_ != NULL) ? master_tim_->CR1 : 0;
    const uint32_t cr2 = (master_tim_ != NULL) ? master_tim_->CR2 : 0;
    Serial.print("CS: diag CFGR=0x");
    Serial.print(cfgr, HEX);
    Serial.print(" ADSTART=");
    Serial.print((cr & ADC_CR_ADSTART) ? 1 : 0);
    Serial.print(" DMA=");
    Serial.print((cfgr & (ADC_CFGR_DMAEN | ADC_CFGR_DMACFG)) >> ADC_CFGR_DMAEN_Pos);
    Serial.print(" running=");
    Serial.print(dma_running_ ? 1 : 0);
    Serial.print(" TIM CR1=0x");
    Serial.print(cr1, HEX);
    Serial.print(" CR2=0x");
    Serial.print(cr2, HEX);
    Serial.print(" buf=");
    Serial.print(sample_buf[0]);
    Serial.print(",");
    Serial.print(sample_buf[1]);
    Serial.print(" DR=");
    Serial.print((uint16_t)LL_ADC_REG_ReadConversionData12(adc_handle.Instance));
    Serial.print(" SQR1=0x");
    Serial.print(adc_handle.Instance->SQR1, HEX);
    if (hdma.Instance != NULL) {
        Serial.print(" CMAR=0x");
        Serial.print((uint32_t)hdma.Instance->CMAR, HEX);
        Serial.print(" buf@=0x");
        Serial.print((uint32_t)(uintptr_t)sample_buf, HEX);
    }
    Serial.println();
}
extern "C" void DMA1_Channel1_IRQHandler(void) {
    if (g_dma1_ch1_hdma != NULL) HAL_DMA_IRQHandler(g_dma1_ch1_hdma);
}
extern "C" void DMA1_Channel2_IRQHandler(void) {
    if (g_dma1_ch2_hdma != NULL) HAL_DMA_IRQHandler(g_dma1_ch2_hdma);
}
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    InlineCurrentSenseSTM32G4* owner = _ownerForAdc(hadc != NULL ? hadc->Instance : NULL);
    if (owner != NULL) owner->notifyConvComplete();
}
#endif // STM32G4xx
