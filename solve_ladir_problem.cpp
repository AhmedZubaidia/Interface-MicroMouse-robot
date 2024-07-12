#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Addresses to assign to the sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Pins to control the shutdown (XSHUT) of the sensors
#define SHT_LOX1 17
#define SHT_LOX2 14

// Motor control pins
#define MOTOR1_PWM 5
#define MOTOR1_IN1 19
#define MOTOR1_IN2 18
#define MOTOR2_PWM 32
#define MOTOR2_IN3 25
#define MOTOR2_IN4 26

// PWM properties
#define PWM_FREQ 1000
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_RESOLUTION 8

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);

  // Configure PWM
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PWM, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR2_PWM, PWM_CHANNEL_2);

  // Initialize shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins initialized..."));

  // Put both sensors in reset mode
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both sensors in reset mode..."));

  // Activate and initialize the sensors
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);
  if (!lox1.begin(LOX1_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  if (!lox2.begin(LOX2_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }

  Serial.println(F("Sensors initialized"));
}

void loop() {
  // Read distances from the sensors
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  int distance1 = (measure1.RangeStatus == 4) ? 1000 : measure1.RangeMilliMeter;
  int distance2 = (measure2.RangeStatus == 4) ? 1000 : measure2.RangeMilliMeter;

  Serial.print("Distance1: ");
  Serial.println(distance1);
  Serial.print("Distance2: ");
  Serial.println(distance2);

  // Check if both distances are less than 80 mm
  if (distance1 < 80 && distance2 < 80) {
    Serial.println("Both distances < 80 mm. Rotating right.");
    rotateRight();
  } else {
    stopMotors();
  }

  delay(100);
}

void rotateRight() {
  // Stop motor 2
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  ledcWrite(PWM_CHANNEL_2, 0);

  // Set motor 1 to rotate the robot to the right
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  ledcWrite(PWM_CHANNEL_1, 125);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
}
