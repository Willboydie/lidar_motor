#include "PWMDriver.hpp"

// init hardware pins
bool PWMDriver::init() {
  // PWM pins
  pinMode(n1, OUTPUT);
  pinMode(n2, OUTPUT);
  pinMode(n3, OUTPUT);
  pinMode(en, OUTPUT);
  

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = configure3PWM(pwm_frequency, n1, n2, n3);

  digitalWrite(en, HIGH);  // Activate the driver

  return params!=nullptr; 
}


void PWMDriver::setPwm(float Va, float Vb, float Vc) {
  // limit the voltage in driver
  Va = _constrain(Va, 0.0f, voltage_limit);
  Vb = _constrain(Vb, 0.0f, voltage_limit);
  Vc = _constrain(Vc, 0.0f, voltage_limit);

  // calculate duty cycle
  // limited in [0,1]
  duty_cycleA = _constrain(Va / voltage_power_supply, 0.0f , 1.0f );
  duty_cycleB = _constrain(Vb / voltage_power_supply, 0.0f , 1.0f );
  duty_cycleC = _constrain(Vc / voltage_power_supply, 0.0f , 1.0f );
  // hardware specific writing
  write3PWM(duty_cycleA, duty_cycleB, duty_cycleC, params);
}


void PWMDriver::disable() {
  if (!enabled) return;
  setPwm(0.0f, 0.0f, 0.0f);
  digitalWrite(en, LOW);
  Serial.println("PWMDriver disabled");
  enabled = false;
}


void PWMDriver::enable() {
  if (enabled) return;
  digitalWrite(en, HIGH);
  Serial.println("PWMDriver enabled");
  enabled = true;
}


void PWMDriver::deinit() {
  disable();
  if (!params) return;

  stopPwmHardware(params);

  pinMode(n1, INPUT);
  pinMode(n2, INPUT);
  pinMode(n3, INPUT);
  pinMode(en, INPUT);

  // Free heap memory
  cleanupParams(params);
  params = nullptr;
}