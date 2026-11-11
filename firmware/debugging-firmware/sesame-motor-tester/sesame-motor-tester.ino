#include <Arduino.h>
#include <Wire.h>

// ======================================================================
// --- PCA9685: External hardware PWM via I2C ---
// ======================================================================
#include <Adafruit_PWMServoDriver.h>

// I2C Pins for ESP32-C3 Super Mini
#define I2C_SDA 8
#define I2C_SCL 10

// PCA9685 driver (address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// ======================================================================
// --- CONFIGURATION ---
// ======================================================================

// PCA9685 servo pulse ticks (12-bit: 0-4095, at 50Hz each tick = ~4.88us)
// For MG90S servo range 732-2929us: 732/4.88=150, 2929/4.88=600
#define SERVO_MIN_TICK  150
#define SERVO_MAX_TICK  600

// PCA9685 channel mapping (channels 0-7)
const int servoChannel[8] = {0, 1, 2, 3, 4, 5, 6, 7};
bool servoEnabled[8] = {false, false, false, false, false, false, false, false};

// ======================================================================
// --- SETUP ---
// ======================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("-----------------------------------");
  Serial.println("   Sesame Motor Tester Interface   ");
  Serial.println("   (ESP32-C3 + PCA9685)            ");
  Serial.println("-----------------------------------");
  Serial.println("Commands:");
  Serial.println("1. id,angle   -> e.g. '0,90'");
  Serial.println("2. all,angle  -> e.g. 'all,90'");
  Serial.println("3. stop       -> Powers down motors");
  Serial.println("-----------------------------------");

  // I2C init
  Wire.begin(I2C_SDA, I2C_SCL);

  // PCA9685 init
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servos
  delay(10);

  // All channels off at start
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servoChannel[i], 0, 0);
  }

  Serial.println("Status: Motors are currently OFF (Limp).");
}

// ======================================================================
// --- MAIN LOOP ---
// ======================================================================

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    if (input.equalsIgnoreCase("stop")) {
      stopMotors();
      return;
    }

    int commaIndex = input.indexOf(',');

    if (commaIndex != -1) {
      String cmd = input.substring(0, commaIndex);
      String valStr = input.substring(commaIndex + 1);
      int angle = valStr.toInt();

      if (angle < 0) angle = 0;
      if (angle > 180) angle = 180;

      if (cmd.equalsIgnoreCase("all")) {
        moveAll(angle);
      } else {
        int motorId = cmd.toInt();
        if (motorId == 0 && cmd.charAt(0) != '0') {
           Serial.println("Error: Invalid Motor ID");
        } else {
           moveMotor(motorId, angle);
        }
      }
    } else {
      Serial.println("Error: Invalid format. Use 'id,angle', 'all,angle', or 'stop'.");
    }
  }
}

// ======================================================================
// --- HELPER FUNCTIONS ---
// ======================================================================

void moveMotor(int id, int angle) {
  if (id < 0 || id > 7) {
    Serial.println("Error: Motor ID must be 0-7");
    return;
  }

  uint16_t tick = map(angle, 0, 180, SERVO_MIN_TICK, SERVO_MAX_TICK);
  pwm.setPWM(servoChannel[id], 0, tick);
  servoEnabled[id] = true;

  Serial.print("OK: Motor ");
  Serial.print(id);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" (tick: ");
  Serial.print(tick);
  Serial.println(")");
}

void moveAll(int angle) {
  Serial.print("Moving ALL to ");
  Serial.println(angle);
  for (int i = 0; i < 8; i++) {
    moveMotor(i, angle);
  }
}

void stopMotors() {
  Serial.println("Stopping all motors...");
  for (int i = 0; i < 8; i++) {
    pwm.setPWM(servoChannel[i], 0, 0);
    servoEnabled[i] = false;
  }
  Serial.println("Motors are now OFF.");
}
