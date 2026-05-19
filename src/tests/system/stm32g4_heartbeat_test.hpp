#pragma once

#include <Arduino.h>

// Stage 0 bench test: 1 Hz LED toggle + serial heartbeat.
// Invoked from setup(); main loop() stays empty.
inline void stm32g4_heartbeat_test() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if defined(USER_BTN)
  pinMode(USER_BTN, INPUT_PULLUP);
  bool last_btn = digitalRead(USER_BTN);
  Serial.println("stm32g4 heartbeat: press USER_BTN to log transitions");
#else
  Serial.println("stm32g4 heartbeat: USER_BTN not defined on this board");
#endif

  Serial.println("stm32g4 heartbeat started");

  unsigned long last_hb_ms = 0;
  bool led_on = false;

  while (true) {
    const unsigned long now = millis();

    if (now - last_hb_ms >= 1000UL) {
      last_hb_ms = now;
      led_on = !led_on;
      digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
      Serial.print("hb ");
      Serial.println(now);
    }

#if defined(USER_BTN)
    const bool btn = digitalRead(USER_BTN);
    if (btn != last_btn) {
      last_btn = btn;
      Serial.print("btn ");
      Serial.println(btn == LOW ? "pressed" : "released");
    }
#endif
  }
}
