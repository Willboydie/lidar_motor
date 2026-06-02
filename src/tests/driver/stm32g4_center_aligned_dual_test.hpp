#pragma once

#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "../../pwm_driver/stm32g4/stm32_mcu.h"
#include "../../pwm_driver/stm32g4/stm32_timerutils.h"
#include "../../utils/constants.h"
#include "Arduino.h"

// Stage 4 bench test: centre-aligned 3PWM + master TRGO on BOTH motors at once.
// Motor X: N1_X=PA8, N2_X=PA9, N3_X=PA10 (TIM1).  Scope TRGO_X on PA6 (D12).
// Motor Y: N1_Y=PC6, N2_Y=PC7, N3_Y=PB9   (TIM8).  Scope TRGO_Y on PB5 (D4).
// Serial: l = 25% on phase A (both), h = 75%, o = default pattern, f = disable, e = re-enable.

#if defined(STM32G4xx)

namespace {

constexpr uint32_t TRGO_PROBE_X_PIN = PA6;  // D12
constexpr uint32_t TRGO_PROBE_Y_PIN = PB5;  // D4

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

void print_master_timer(const char* label, TIM_TypeDef* tim) {
  Serial.print(label);
  Serial.print(" master_timer=TIM");
#if defined(SIMPLEFOC_STM32_DEBUG)
  Serial.println(stm32_getTimerNumber(tim));
#else
  Serial.println(reinterpret_cast<uintptr_t>(tim), HEX);
#endif
  if (tim != NULL) {
    const uint32_t trgo = (tim->CR2 >> TIM_CR2_MMS_Pos) & 0x7U;
    Serial.print("  TRGO mode: ");
    if (trgo == LL_TIM_TRGO_UPDATE) {
      Serial.println("UPDATE");
    } else {
      Serial.print("MMS=");
      Serial.println(trgo);
    }
    if (IS_TIM_REPETITION_COUNTER_INSTANCE(tim)) {
      Serial.print("  RCR=");
      Serial.println(static_cast<int>(tim->RCR));
    }
  }
}

}  // namespace

inline void stm32g4_center_aligned_dual_test() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  delay(500);
  Serial.println("stm32g4 centre-aligned + TRGO test (motors X + Y) starting...");

  PWMDriverSTM32G4 driverX(N1_X, N2_X, N3_X, EN_X);
  if (!driverX.init()) {
    Serial.println("Failed to initialize motor X PWM driver");
    return;
  }
  PWMDriverSTM32G4 driverY(N1_Y, N2_Y, N3_Y, EN_Y);
  if (!driverY.init()) {
    Serial.println("Failed to initialize motor Y PWM driver");
    return;
  }

  driverX.enable();
  driverY.enable();

  print_master_timer("X:", driverX.getMasterTimerInstance());
  print_master_timer("Y:", driverY.getMasterTimerInstance());
  Serial.print("motorsUsed=");
  Serial.println(stm32_getNumMotorsUsed());

  enable_trgo_probes(driverX.getMasterTimerHandle(), driverY.getMasterTimerHandle());

  driverX.setPwm(3.0f, 1.5f, 0.0f);
  driverY.setPwm(3.0f, 1.5f, 0.0f);

  // TIM1 (X) and TIM8 (Y) are independent masters started sequentially during
  // init, so their counters — and TRGO events — are skewed by microseconds.
  // Phase-align them so both motors' PWM edges and TRGO pulses coincide.
  TIM_HandleTypeDef* masters[2] = { driverX.getMasterTimerHandle(), driverY.getMasterTimerHandle() };
  stm32_syncTimers(masters, 2);
  g_trgo_x_count = 0;
  g_trgo_y_count = 0;

  Serial.println("PWM ON (both): Va=3.0 V (25%), Vb=1.5 V, Vc=0.0 V");
  Serial.println("Scope TRGO_X on PA6 (D12), TRGO_Y on PB5 (D4). Commands:");
  Serial.println("  l = 25% on A (3.0 V), h = 75% on A (9.0 V)");
  Serial.println("  o = default (3/1.5/0 V), f = disable, e = enable + default");

  while (true) {
    poll_trgo_probes();

    if (Serial.available() > 0) {
      const char cmd = static_cast<char>(Serial.read());
      switch (cmd) {
        case 'l':
        case 'L':
          driverX.setPwm(3.0f, 0.0f, 0.0f);
          driverY.setPwm(3.0f, 0.0f, 0.0f);
          Serial.println("duty: Va=25% (3.0 V), Vb=Vc=0 (both)");
          break;
        case 'h':
        case 'H':
          driverX.setPwm(9.0f, 0.0f, 0.0f);
          driverY.setPwm(9.0f, 0.0f, 0.0f);
          Serial.println("duty: Va=75% (9.0 V), Vb=Vc=0 (both)");
          break;
        case 'o':
        case 'O':
        case '1':
          driverX.enable();
          driverY.enable();
          driverX.setPwm(3.0f, 1.5f, 0.0f);
          driverY.setPwm(3.0f, 1.5f, 0.0f);
          Serial.println("PWM ON (both): Va=3.0 V, Vb=1.5 V, Vc=0.0 V");
          break;
        case 'f':
        case 'F':
        case '0':
          driverX.disable();
          driverY.disable();
          Serial.println("PWM OFF: both drivers disabled");
          break;
        case 'e':
        case 'E':
          driverX.enable();
          driverY.enable();
          driverX.setPwm(3.0f, 1.5f, 0.0f);
          driverY.setPwm(3.0f, 1.5f, 0.0f);
          Serial.println("PWM ON: both re-enabled");
          break;
        case '\r':
        case '\n':
          break;
        default:
          Serial.println("Unknown command. Use l, h, o, f, or e.");
          break;
      }
    }

    static unsigned long last_status_ms = 0;
    const unsigned long now = millis();
    if (now - last_status_ms >= 2000UL) {
      last_status_ms = now;
      Serial.print("trgo_x=");
      Serial.print(g_trgo_x_count);
      Serial.print("  trgo_y=");
      Serial.println(g_trgo_y_count);
    }
  }
}

#endif
