#include <Arduino.h>
#include <SimpleFOC.h>

// Motor 1 (Pitch axis) - Gimbal Motor 1
BLDCMotor motor1 = BLDCMotor(7); // 7 pole pairs for typical gimbal motor
BLDCDriver3PWM driver1 = BLDCDriver3PWM(2, 3, 4, 1); // PWM pins for motor 1

// Motor 2 (Yaw axis) - Gimbal Motor 2  
BLDCMotor motor2 = BLDCMotor(7); // 7 pole pairs for typical gimbal motor
BLDCDriver3PWM driver2 = BLDCDriver3PWM(5, 6, 7, 1); // PWM pins for motor 2

// AS5048 Magnetic Encoders
// Motor 1 AS5048 (Pitch axis) - PWM output
MagneticSensorPWM sensor1 = MagneticSensorPWM(8, 4, 904); // PWM pin, pulse count, max angle
// Motor 2 AS5048 (Yaw axis) - PWM output  
MagneticSensorPWM sensor2 = MagneticSensorPWM(9, 4, 904); // PWM pin, pulse count, max angle

// Control variables
float target_angle1 = 0.0; // Target angle for motor 1 (degrees)
float target_angle2 = 0.0; // Target angle for motor 2 (degrees)
float current_angle1 = 0.0; // Current angle for motor 1 (degrees)
float current_angle2 = 0.0; // Current angle for motor 2 (degrees)

// Serial communication
String inputString = "";
bool stringComplete = false;

// Debug timing
unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 100; // Debug every 100ms

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial to initialize
  
  Serial.println("=== Dual Gimbal Motor Controller with AS5048 ===");
  Serial.println("Initializing motors with magnetic encoders...");
  
  // Initialize Motor 1
  Serial.println("Setting up Motor 1 (Pitch axis) with AS5048...");
  
  // Configure AS5048 sensor 1
  sensor1.init();
  Serial.print("AS5048 Sensor 1 initialized. Current angle: ");
  Serial.print(sensor1.getAngle() * 180.0 / PI, 2);
  Serial.println(" degrees");
  
  // Link sensor to motor 1
  motor1.linkSensor(&sensor1);
  
  // Configure driver 1
  driver1.voltage_power_supply = 12.0; // 12V power supply
  driver1.init();
  motor1.linkDriver(&driver1);
  
  // Configure motor 1 parameters for gimbal application
  motor1.voltage_sensor_align = 1.0; // Lower voltage for sensor alignment
  motor1.velocity_limit = 50.0; // Limit velocity (rad/s)
  motor1.voltage_limit = 3.0; // Limit voltage for smooth operation
  
  // Initialize motor 1
  motor1.init();
  motor1.initFOC();
  
  Serial.println("Motor 1 initialized successfully!");
  
  // Initialize Motor 2
  Serial.println("Setting up Motor 2 (Yaw axis) with AS5048...");
  
  // Configure AS5048 sensor 2
  sensor2.init();
  Serial.print("AS5048 Sensor 2 initialized. Current angle: ");
  Serial.print(sensor2.getAngle() * 180.0 / PI, 2);
  Serial.println(" degrees");
  
  // Link sensor to motor 2
  motor2.linkSensor(&sensor2);
  
  // Configure driver 2
  driver2.voltage_power_supply = 12.0; // 12V power supply
  driver2.init();
  motor2.linkDriver(&driver2);
  
  // Configure motor 2 parameters for gimbal application
  motor2.voltage_sensor_align = 1.0; // Lower voltage for sensor alignment
  motor2.velocity_limit = 50.0; // Limit velocity (rad/s)
  motor2.voltage_limit = 3.0; // Limit voltage for smooth operation
  
  // Initialize motor 2
  motor2.init();
  motor2.initFOC();
  
  Serial.println("Motor 2 initialized successfully!");
  
  // Set control mode to angle control
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;
  
  // Set target to current position (hold position)
  motor1.target = motor1.shaft_angle;
  motor2.target = motor2.shaft_angle;
  
  Serial.println("=== System Ready ===");
  Serial.println("AS5048 Magnetic Encoders Active");
  Serial.println("Commands:");
  Serial.println("  set1 <angle> - Set motor 1 target angle (degrees)");
  Serial.println("  set2 <angle> - Set motor 2 target angle (degrees)");
  Serial.println("  home1 - Home motor 1 to 0 degrees");
  Serial.println("  home2 - Home motor 2 to 0 degrees");
  Serial.println("  home - Home both motors to 0 degrees");
  Serial.println("  stop - Stop both motors");
  Serial.println("  status - Show current status");
  Serial.println("  help - Show this help");
  Serial.println();
}

void loop() {
  // Handle serial communication
  handleSerial();
  
  // Run motor control loops
  motor1.loopFOC();
  motor2.loopFOC();
  
  // Update current angles for display
  current_angle1 = motor1.shaft_angle * 180.0 / PI;
  current_angle2 = motor2.shaft_angle * 180.0 / PI;
  
  // Debug output
  if (millis() - lastDebugTime >= debugInterval) {
    printStatus();
    lastDebugTime = millis();
  }
}

void handleSerial() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  if (command.startsWith("set1 ")) {
    float angle = command.substring(5).toFloat();
    setMotor1Target(angle);
  }
  else if (command.startsWith("set2 ")) {
    float angle = command.substring(5).toFloat();
    setMotor2Target(angle);
  }
  else if (command == "home1") {
    homeMotor1();
  }
  else if (command == "home2") {
    homeMotor2();
  }
  else if (command == "home") {
    homeBothMotors();
  }
  else if (command == "stop") {
    stopMotors();
  }
  else if (command == "status") {
    printDetailedStatus();
  }
  else if (command == "help") {
    printHelp();
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

void setMotor1Target(float angle) {
  target_angle1 = angle;
  motor1.target = angle * PI / 180.0; // Convert to radians
  Serial.print("Motor 1 target set to: ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void setMotor2Target(float angle) {
  target_angle2 = angle;
  motor2.target = angle * PI / 180.0; // Convert to radians
  Serial.print("Motor 2 target set to: ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void homeMotor1() {
  motor1.target = 0.0;
  target_angle1 = 0.0;
  Serial.println("Motor 1 homing to 0 degrees");
}

void homeMotor2() {
  motor2.target = 0.0;
  target_angle2 = 0.0;
  Serial.println("Motor 2 homing to 0 degrees");
}

void homeBothMotors() {
  motor1.target = 0.0;
  motor2.target = 0.0;
  target_angle1 = 0.0;
  target_angle2 = 0.0;
  Serial.println("Both motors homing to 0 degrees");
}

void stopMotors() {
  motor1.disable();
  motor2.disable();
  Serial.println("Motors stopped");
}

void printStatus() {
  // Only print basic status every debug interval
  static int statusCounter = 0;
  statusCounter++;
  
  if (statusCounter % 10 == 0) { // Print every 1 second (10 * 100ms)
    Serial.print("M1: ");
    Serial.print(current_angle1, 1);
    Serial.print("° (target: ");
    Serial.print(target_angle1, 1);
    Serial.print("°) | M2: ");
    Serial.print(current_angle2, 1);
    Serial.print("° (target: ");
    Serial.print(target_angle2, 1);
    Serial.println("°)");
  }
}

void printDetailedStatus() {
  Serial.println("=== Detailed Status ===");
  Serial.print("Motor 1 (Pitch): ");
  Serial.print("Angle: ");
  Serial.print(current_angle1, 2);
  Serial.print("°, Target: ");
  Serial.print(target_angle1, 2);
  Serial.print("°, Velocity: ");
  Serial.print(motor1.shaft_velocity * 180.0 / PI, 2);
  Serial.println("°/s");
  
  Serial.print("Motor 2 (Yaw): ");
  Serial.print("Angle: ");
  Serial.print(current_angle2, 2);
  Serial.print("°, Target: ");
  Serial.print(target_angle2, 2);
  Serial.print("°, Velocity: ");
  Serial.print(motor2.shaft_velocity * 180.0 / PI, 2);
  Serial.println("°/s");
  
  Serial.print("Motor 1 enabled: ");
  Serial.println(motor1.enabled ? "Yes" : "No");
  Serial.print("Motor 2 enabled: ");
  Serial.println(motor2.enabled ? "Yes" : "No");
  
  // AS5048 specific status
  Serial.print("AS5048 Sensor 1: ");
  Serial.print(sensor1.getAngle() * 180.0 / PI, 2);
  Serial.println("°");
  Serial.print("AS5048 Sensor 2: ");
  Serial.print(sensor2.getAngle() * 180.0 / PI, 2);
  Serial.println("°");
  Serial.println();
}

void printHelp() {
  Serial.println("=== Available Commands ===");
  Serial.println("set1 <angle>  - Set motor 1 target angle (degrees)");
  Serial.println("set2 <angle>  - Set motor 2 target angle (degrees)");
  Serial.println("home1         - Home motor 1 to 0 degrees");
  Serial.println("home2         - Home motor 2 to 0 degrees");
  Serial.println("home          - Home both motors to 0 degrees");
  Serial.println("stop          - Stop both motors");
  Serial.println("status        - Show detailed status");
  Serial.println("help          - Show this help");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  set1 45.5   - Move motor 1 to 45.5 degrees");
  Serial.println("  set2 -30    - Move motor 2 to -30 degrees");
  Serial.println("  home        - Home both motors");
  Serial.println();
  Serial.println("AS5048 Features:");
  Serial.println("  - Absolute position sensing");
  Serial.println("  - No homing required on startup");
  Serial.println("  - High precision (14-bit resolution)");
  Serial.println("  - Robust magnetic sensing");
  Serial.println();
}