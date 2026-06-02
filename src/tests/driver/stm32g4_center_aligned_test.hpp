#pragma once

#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "../../pwm_driver/stm32g4/stm32_mcu.h"
#include "../../pwm_driver/stm32g4/stm32_timerutils.h"
#include "../../utils/constants.h"
#include "Arduino.h"

// Stage 4 bench test: centre-aligned 3PWM + master TRGO on motor X (no coils).
// Scope TRGO_PROBE_PIN (PC7 / D9) for one toggle per PWM period at count-zero.
// Serial: l = 25% duty on phase A, h = 75%, o = default pattern, f = disable, e = re-enable.

#if defined(STM32G4xx)

namespace {

constexpr uint32_t TRGO_PROBE_PIN = PC7;

TIM_HandleTypeDef* g_master_timer = NULL;
volatile uint32_t g_trgo_toggle_count = 0;

void trgo_probe_toggle() {
  digitalWrite(TRGO_PROBE_PIN, digitalRead(TRGO_PROBE_PIN) ? LOW : HIGH);
  g_trgo_toggle_count++;
}

void enable_trgo_probe(TIM_HandleTypeDef* master) {
  g_master_timer = master;
  pinMode(TRGO_PROBE_PIN, OUTPUT);
  digitalWrite(TRGO_PROBE_PIN, LOW);
}

void poll_trgo_probe() {
  if (g_master_timer == NULL) return;
  if (__HAL_TIM_GET_FLAG(g_master_timer, TIM_FLAG_UPDATE) != RESET) {
    __HAL_TIM_CLEAR_FLAG(g_master_timer, TIM_FLAG_UPDATE);
    trgo_probe_toggle();
  }
}

void print_master_timer(TIM_TypeDef* tim) {
  Serial.print("master_timer=TIM");
#if defined(SIMPLEFOC_STM32_DEBUG)
  Serial.println(stm32_getTimerNumber(tim));
#else
  Serial.println(reinterpret_cast<uintptr_t>(tim), HEX);
#endif
  Serial.print("TRGO mode: ");
  if (tim != NULL) {
    const uint32_t trgo = (tim->CR2 >> TIM_CR2_MMS_Pos) & 0x7U;
    if (trgo == LL_TIM_TRGO_UPDATE) {
      Serial.println("UPDATE");
    } else {
      Serial.print("MMS=");
      Serial.println(trgo);
    }
  }
  if (tim != NULL && IS_TIM_REPETITION_COUNTER_INSTANCE(tim)) {
    Serial.print("RCR=");
    Serial.println(static_cast<int>(tim->RCR));
  }
}

}  // namespace

inline void stm32g4_center_aligned_test() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  delay(500);
  Serial.println("stm32g4 centre-aligned + TRGO test starting...");

  PWMDriverSTM32G4 driver(N1_X, N2_X, N3_X, EN_X);
  if (!driver.init()) {
    Serial.println("Failed to initialize PWM driver");
    return;
  }

  driver.enable();
  TIM_HandleTypeDef* master = driver.getMasterTimerHandle();
  print_master_timer(driver.getMasterTimerInstance());
  enable_trgo_probe(master);

  driver.setPwm(3.0f, 1.5f, 0.0f);
  Serial.println("PWM ON: Va=3.0 V (25%), Vb=1.5 V, Vc=0.0 V");
  Serial.println("Scope TRGO_PROBE on PC7 (D9). Commands:");
  Serial.println("  l = 25% on A (3.0 V), h = 75% on A (9.0 V)");
  Serial.println("  o = default (3/1.5/0 V), f = disable, e = enable + default");

  while (true) {
    poll_trgo_probe();

    if (Serial.available() > 0) {
      const char cmd = static_cast<char>(Serial.read());
      switch (cmd) {
        case 'l':
        case 'L':
          driver.setPwm(3.0f, 0.0f, 0.0f);
          Serial.println("duty: Va=25% (3.0 V), Vb=Vc=0");
          break;
        case 'h':
        case 'H':
          driver.setPwm(9.0f, 0.0f, 0.0f);
          Serial.println("duty: Va=75% (9.0 V), Vb=Vc=0");
          break;
        case 'o':
        case 'O':
        case '1':
          driver.setPwm(3.0f, 1.5f, 0.0f);
          driver.enable();
          Serial.println("PWM ON: Va=3.0 V, Vb=1.5 V, Vc=0.0 V");
          break;
        case 'f':
        case 'F':
        case '0':
          driver.disable();
          Serial.println("PWM OFF: driver disabled");
          break;
        case 'e':
        case 'E':
          driver.enable();
          driver.setPwm(3.0f, 1.5f, 0.0f);
          Serial.println("PWM ON: re-enabled");
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
      Serial.print("trgo_toggles=");
      Serial.println(g_trgo_toggle_count);
    }
  }
}

#endif
