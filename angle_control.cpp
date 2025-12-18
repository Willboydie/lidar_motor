#include "angle_control.hpp"


// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void angle_control_test() {
    // magnetic sensor instance - SPI
    MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, SENSOR_SPI_CS);

    // BLDC motor & driver instance
    BLDCMotor motor = BLDCMotor(POLE_PAIRS);
    BLDCDriver3PWM driver = BLDCDriver3PWM(N1, N2, N3, EN);

    // use monitoring with serial 
    Serial.begin(115200);
    delay(1000);

    // enable more verbose output for debugging
    SimpleFOCDebug::enable(&Serial);

    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // choose FOC modulation (optional)
    motor.foc_modulation = ;

    motor.torque_controller = TORQUE_CONTROLLER;

    // set motion control loop to be used
    motor.controller = CONTROLLER;

    // contoller configuration 
    // default parameters in defaults.h 
    // velocity PI controller parameters 
    motor.PID_velocity.P = PID_VELOCITY_P; 
    motor.PID_velocity.I = PID_VELOCITY_I; 
    motor.PID_velocity.D = PID_VELOCITY_D; 
    // maximal voltage to be set to the motor 
    motor.voltage_limit = VOLTAGE_LIMIT; 
    motor.current_limit = CURRENT_LIMIT;
    // velocity low pass filtering time constant 
    // the lower the less filtered 
    motor.LPF_velocity.Tf = LPF_VELOCITY_TF; 
    // angle P controller 
    motor.P_angle.P = P_ANGLE_P; 
    // maximal velocity of the position control 
    motor.velocity_limit = VELOCITY_LIMIT;

    // motor.sensor_direction = SENSOR_DIRECTION;

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // add target command T
    command.add('X', doTarget, "target angle");

    
    while (true) {
        // main FOC algorithm function
        // the faster you run this function the better
        // Arduino UNO loop  ~1kHz
        // Bluepill loop ~10kHz
        motor.loopFOC();

        // Motion control function
        // velocity, position or voltage (defined in motor.controller)
        // this function can be run at much lower frequency than loopFOC() function
        // You can also use motor.move() and set the motor.target in the code
        motor.move(target_angle);

        static unsigned long time = 0;
        if (millis() - time > 100) {
            time = millis();
            Serial.print("Sensor angle: " + String(sensor.getSensorAngle()));
            Serial.print("    shaft angle: " + String(motor.shaft_angle));
            Serial.print("    target: " + String(motor.shaft_angle_sp));
            Serial.print("    sought velocity: " + String(motor.shaft_velocity_sp));
            Serial.print("    sought current: " + String(motor.current_sp));
            Serial.print("    set voltage q: " + String(motor.voltage.q));
            Serial.print("    set voltage d: " + String(motor.voltage.d));
            Serial.println("    electrical angle: " + String(motor.electrical_angle));
        }

        // user communication
        command.run();
    }
}
