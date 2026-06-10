#include "InlineCurrentSenseSTM32G4.hpp"
#if defined(STM32G4xx)
#include "PeripheralPins.h"
#include "pinmap.h"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "pwm_driver/stm32g4/stm32_timerutils.h"
#include "utils/quick_defs.h"
#include "stm32g4xx_ll_tim.h"
static constexpr uint32_t kAdcPollTimeoutMs = 100;
static constexpr uint32_t kAdcSampleTime = ADC_SAMPLETIME_47CYCLES_5;
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
static void _adcPinInitForAdc(PinName pin, ADC_TypeDef* adc) {
    for (int i = 0; PinMap_ADC[i].pin != NC; i++) {
        if (pin != (PinMap_ADC[i].pin & ~ALTX_MASK)) continue;
        if ((ADC_TypeDef*)PinMap_ADC[i].peripheral != adc) continue;
        pinmap_pinout(PinMap_ADC[i].pin, PinMap_ADC);
        return;
    }
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
static void _tearDownDma(ADC_HandleTypeDef& hadc, DMA_HandleTypeDef& hdma) {
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
    ADC_TypeDef* adc
) {
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.SamplingTime = kAdcSampleTime;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.Channel = _adcChannelForPin(pn_a, adc);
    if (sConfig.Channel == 0xFFFFFFFF || HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Serial.println("CS: ERR: cannot configure channel A");
        return false;
    }
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.Channel = _adcChannelForPin(pn_b, adc);
    if (sConfig.Channel == 0xFFFFFFFF || HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Serial.println("CS: ERR: cannot configure channel B");
        return false;
    }
    return true;
}
// ---------- InlineCurrentSenseSTM32G4 ----------
InlineCurrentSenseSTM32G4::InlineCurrentSenseSTM32G4(float mVpA, int pinA_, int pinB_, PWMDriverSTM32G4& _driver)
: gain_a(1000.0f / mVpA)
, gain_b(1000.0f / mVpA)
, gain_c(1000.0f / mVpA)
, offset_ia(1.65f)
, offset_ib(1.65f)
, offset_ic(0.0f)
, pinA(pinA_)
, pinB(pinB_)
, driver(_driver)
{}
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
bool InlineCurrentSenseSTM32G4::init(bool hardware_trigger) {
    PinName pn_a = analogInputToPinName(pinA);
    PinName pn_b = analogInputToPinName(pinB);
    if (pn_a == NC || pn_b == NC) {
        Serial.println("CS: ERR: current sense pin is not an analog pin");
        return false;
    }
    ADC_TypeDef* adc = _findCommonADC(pn_a, pn_b);
    if (adc == nullptr) {
        Serial.println("CS: ERR: no common ADC for the current sense pins");
        return false;
    }
    _adcClockEnable(adc);
    _adcPinInitForAdc(pn_a, adc);
    _adcPinInitForAdc(pn_b, adc);
    TIM_HandleTypeDef* master = driver.getMasterTimerHandle();
    uint32_t trgo = 0xFFFFFFFF;
    if (hardware_trigger && master != NULL) {
        trgo = _timerToRegularTRGO(master->Instance);
    }
    hardware_dma_ = hardware_trigger && (master != NULL) && (trgo != 0xFFFFFFFF);
    _tearDownDma(adc_handle, hdma);
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
    if (hardware_dma_) {
        _applyHardwareAdcFields(adc_handle, trgo);
    } else {
        _applySoftwareAdcFields(adc_handle);
    }
    if (HAL_ADC_Init(&adc_handle) != HAL_OK) {
        Serial.print("CS: ERR: HAL_ADC_Init failed, err=0x");
        Serial.println(adc_handle.ErrorCode, HEX);
        return false;
    }
    if (!_configureAdcChannels(adc_handle, pn_a, pn_b, adc)) {
        return false;
    }
    if (HAL_ADCEx_Calibration_Start(&adc_handle, ADC_SINGLE_ENDED) != HAL_OK) {
        Serial.println("CS: ERR: ADC self-calibration failed");
        return false;
    }
    if (hardware_dma_) {
        if (!_configureDmaForAdc(adc_handle, hdma) ||
            HAL_ADC_Start_DMA(&adc_handle, (uint32_t*)sample_buf, 2) != HAL_OK) {
            Serial.println("CS: WARN: TRGO+DMA failed; falling back to software scan");
            hardware_dma_ = false;
            _tearDownDma(adc_handle, hdma);
            _resetAdcHandle(adc_handle, adc);
            _applySoftwareAdcFields(adc_handle);
            if (HAL_ADC_Init(&adc_handle) != HAL_OK) {
                Serial.println("CS: ERR: software fallback HAL_ADC_Init failed");
                return false;
            }
            if (!_configureAdcChannels(adc_handle, pn_a, pn_b, adc)) {
                return false;
            }
            if (HAL_ADCEx_Calibration_Start(&adc_handle, ADC_SINGLE_ENDED) != HAL_OK) {
                Serial.println("CS: ERR: software fallback calibration failed");
                return false;
            }
        }
    }
    if (hardware_dma_) {
        Serial.print("CS: TRGO+DMA on ADC");
        Serial.print((adc == ADC1) ? 1 : (adc == ADC2) ? 2 : 0);
        Serial.print(", master TIM");
        Serial.println(stm32_getTimerNumber(master->Instance));
    } else {
        Serial.print("CS: software scan on ADC");
        Serial.print((adc == ADC1) ? 1 : (adc == ADC2) ? 2 : 0);
        Serial.print(" rank1=IN");
        Serial.print((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_a, adc)));
        Serial.print(" rank2=IN");
        Serial.println((int)__LL_ADC_CHANNEL_TO_DECIMAL_NB(_adcChannelForPin(pn_b, adc)));
    }
    initialised = true;
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
DQCurrent_s InlineCurrentSenseSTM32G4::getFOCCurrents(float angle_el) {
    PhaseCurrent_s current = getPhaseCurrents();
    float i_alpha = current.a;
    float i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    float st, ct;
    fast_sincos(angle_el, &st, &ct);
    DQCurrent_s dq;
    dq.d = i_alpha * ct + i_beta * st;
    dq.q = i_beta * ct - i_alpha * st;
    return dq;
}
bool InlineCurrentSenseSTM32G4::readRawCodes(uint16_t& rawA, uint16_t& rawB) {
    if (!initialised) return false;
    if (!hardware_dma_ && !_pollSampleBuf()) return false;
    rawA = sample_buf[0];
    rawB = sample_buf[1];
    return true;
}
#endif // STM32G4xx
