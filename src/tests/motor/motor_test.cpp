#include "motor_test.hpp"

void motor_test() {

    // BLDC motor & driver instance
    BLDCMotor motor = BLDCMotor(POLE_PAIRS);
    BLDCDriver3PWM driver = BLDCDriver3PWM(N1, N2, N3, EN);

    // use monitoring with serial 
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Motor Open-Loop Test Script Started.");

    // --- Driver Setup ---
    driver.voltage_limit = MOTOR_TEST_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);

    // --- Motor Setup ---
    // We are not using a sensor, so we set the control mode to open-loop velocity
    motor.controller = MotionControlType::velocity_openloop; 
    motor.velocity_limit = MOTOR_TEST_TARGET_VELOCITY;
    motor.voltage_limit = MOTOR_TEST_VOLTAGE_LIMIT;

    // Initialize motor (enables the driver)
    motor.init();
    
    // Set initial voltage/current to start the motor
    motor.target = MOTOR_TEST_TARGET_VELOCITY; 

    Serial.println("Motor spinning commanded. Check rotation.");

    static unsigned long time0 = millis();
    while (millis() - time0 < 10000) {

        // LoopFOC is necessary even for open-loop, as it calls the velocityOpenloop() function.
        // This function iteratively calculates the next electrical angle and sets the voltage.
        motor.loopFOC();

        // Move function runs the controller (in this case, it's the Open-Loop logic)
        motor.move(); 

        // Optional: Change direction every 5 seconds
        if (millis() % 10000 < 5000) {
        motor.target = MOTOR_TEST_TARGET_VELOCITY;
        } else {
        motor.target = -MOTOR_TEST_TARGET_VELOCITY; // Spin CCW
        }

    }
}