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

#define IR_Front_PIN2 27



// 

#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include "Arduino.h"
#include <Ticker.h>  // Library for managing timers


#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Addresses to assign to the sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Pins to control the shutdown (XSHUT) of the sensors
#define SHT_LOX1 17
#define SHT_LOX2 14

// Sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data structures
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;


// Define PWM pins for motor speed control
#define MOTOR1_PWM 5
#define MOTOR2_PWM 32

#define MOTOR1_IN1 18
#define MOTOR1_IN2 19
#define MOTOR2_IN3 25
#define MOTOR2_IN4 26

// Define encoder pins
#define ENCODER1_C1 34
#define ENCODER1_C2 35
#define ENCODER2_C1 4
#define ENCODER2_C2 16


volatile bool conditionMet = false;             // Flag to track if the condition has been met
volatile unsigned long conditionStartTime = 0;  // Time when the condition started

volatile bool resetFlag = false;           // Flag to indicate reset condition
volatile unsigned long lastCheckTime = 0;  // Time when the last check occurred

volatile int lastDiff = 0;  // Global variable to store the last difference value

int motorIncrement2_both =0 ;
int motorIncrement1_both = 0; 

volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool initializationCompleted = false;

int motor1Speed = 100;
int motor2Speed = 120;
unsigned long lastChangeTime1 = 0;
volatile unsigned long lastChangeTime2 = 0;
volatile int lastEnc1Pos = 0;
volatile int lastEnc2Pos = 0;
int last_increment2;
int last_increment1;
volatile unsigned long elapsedMillis = 0;

volatile unsigned long globalElapsedMillis = 0;

volatile int motorIncrement1 = 0;         // Global variable to store the increment value for motor 1
volatile int motorIncrement2 = 0;         // Global variable to store the increment value for motor 2
volatile unsigned long lastDiffTime = 0;  // Global variable to track the time of the last difference

int temp1_speed=90; //basic speed for correction 
int temp2_speed=90;

unsigned long previousMillis = 0;
const long interval = 10;  // interval to read sensors (10 milliseconds)
volatile bool irFrontISRLow = false;  // Flag for IR2 being low


// EEPROM addresses for saving speeds
#define EEPROM_ADDR_MOTOR1_SPEED 0
#define EEPROM_ADDR_MOTOR2_SPEED 4

VL53L0X sensor;

Ticker encoderTicker;   // Create a Ticker object
Ticker distanceTicker;  // Create a Ticker object
Ticker timeTicker;      // Create a Ticker object for timing



// Define timer
hw_timer_t * timer = NULL;
volatile unsigned long elapsedTime = 0;
volatile bool timeFlags[8] = {false};
volatile bool leftWheelFlags[8] = {false};
volatile bool rightWheelFlags[8] = {false};
volatile bool bothWheelsFlags[8] = {false};


volatile long lastEncoder1Pos = 0;
volatile long lastEncoder2Pos = 0;



void IRAM_ATTR onTimer() {
  elapsedTime += 1;  // Increment elapsed time by 1 ms

  // Set flags for every 0.5 seconds (500 ms)
  if (elapsedTime % 100 == 0) {
    int index = (elapsedTime / 100) - 1;
    if (index >= 0 && index < 8) {
      if ( abs(encoder1Pos - lastEncoder1Pos) <5) {
        leftWheelFlags[index] = true;
      } else {
        leftWheelFlags[index] = false;
      }

      if ( abs (encoder2Pos - lastEncoder2Pos) <5) {
        rightWheelFlags[index] = true;
      } else {
        rightWheelFlags[index] = false;
      }

      if ( abs(encoder1Pos - lastEncoder1Pos)<5  && abs(encoder2Pos - lastEncoder2Pos) <5) {
        bothWheelsFlags[index] = true;
      } else {
        bothWheelsFlags[index] = false;
      }

      lastEncoder1Pos = encoder1Pos;
      lastEncoder2Pos = encoder2Pos;
    }
  }

  // Reset elapsedTime after 4000 ms
  if (elapsedTime == 6000) {
    elapsedTime = 0;
    // Reset all flags
    for (int i = 0; i < 8; i++) {
      leftWheelFlags[i] = false;
      rightWheelFlags[i] = false;
      bothWheelsFlags[i] = false;
    }
  }
}


void IRAM_ATTR updateEncoder1C1() {
  int A = digitalRead(ENCODER1_C1);
  int B = digitalRead(ENCODER1_C2);
  portENTER_CRITICAL_ISR(&mux);
  if (A == B) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR updateEncoder1C2() {
  int A = digitalRead(ENCODER1_C1);
  int B = digitalRead(ENCODER1_C2);
  portENTER_CRITICAL_ISR(&mux);
  if (A != B) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR updateEncoder2C1() {
  int A = digitalRead(ENCODER2_C1);
  int B = digitalRead(ENCODER2_C2);
  portENTER_CRITICAL_ISR(&mux);
  if (A == B) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR updateEncoder2C2() {
  int A = digitalRead(ENCODER2_C1);
  int B = digitalRead(ENCODER2_C2);
  portENTER_CRITICAL_ISR(&mux);
  if (A != B) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR irFrontISR() {
  portENTER_CRITICAL_ISR(&mux);
 irFrontISRLow = digitalRead(IR_Front_PIN2) == LOW;
  portEXIT_CRITICAL_ISR(&mux);
}


void updateElapsedMillis() {
  globalElapsedMillis++;
}


void setup() {
  Serial.begin(115200);


    // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_C1), updateEncoder1C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_C2), updateEncoder1C2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_C1), updateEncoder2C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_C2), updateEncoder2C2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_Front_PIN2), irFrontISR, CHANGE);


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

    // Attach other tickers and timers as needed
  timeTicker.attach(0.001, updateElapsedMillis);  // Increment globalElapsedMillis every 1 ms

  timer = timerBegin(0, 80, true);              // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);  // Attach ISR to timer
  timerAlarmWrite(timer, 1000, true);           // Set alarm to 1 ms (80 MHz / 80 = 1 MHz, so 1000 counts = 1 ms)
  timerAlarmEnable(timer);                      // Enable the alarm

  Serial.println(F("Sensors initialized"));
}

int isInitialized  = false;
void loop() {
  // Read distances from the sensors
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  int distance1 = (measure1.RangeStatus == 4) ? 1000 : measure1.RangeMilliMeter;
  int distance2 = (measure2.RangeStatus == 4) ? 1000 : measure2.RangeMilliMeter;

  distance1 = distance1 -30; 
  distance2 = distance2 -30 ;

  Serial.print("Distance1: ");
  Serial.println(distance1);
  Serial.print("Distance2: ");
  Serial.println(distance2);

  portENTER_CRITICAL(&mux);
  bool ir_front_detect = irFrontISRLow; // Right sensor
  portEXIT_CRITICAL(&mux);

  if(!isInitialized ){

    stopMotors();
    delay(700);
    isInitialized  =true; 

  }




if (ir_front_detect) {

 // Check if both distances are less than 80 mm
 if (distance1 > 80 && distance2 < 80 ) { 
      stopMotors();
      Serial.println("Going right.");
      rotateMotor1ToRight();
      stopMotors();


  }
    if (distance2 > 80 && distance1 < 80) { 
    stopMotors();
    Serial.println("Going left.");
    rotateMotor2ToLeft();
    stopMotors();


  } 

 if (distance1 > 80 && distance2 > 80) {
      stopMotors();

      Serial.println("Going right.");
    rotateMotor1ToRight();
    stopMotors();
  }

  if (distance1 < 80 && distance2 < 80) {
      Serial.println("Turning around.");
       stopMotors();
      turnAround();
      stopMotors();


  } 



}else{

  forward_with_correction(motor1Speed,motor2Speed);
}
 
 // delay(100);
}


void forward_with_correction(int &motor1Speed, int &motor2Speed) {
    int enc1Pos, enc2Pos;
    int diff;

    // Set motor directions
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);

    // Read encoder values
    portENTER_CRITICAL(&mux);
    enc1Pos = encoder1Pos;
    enc2Pos = encoder2Pos;
    portEXIT_CRITICAL(&mux);

    diff = enc1Pos - enc2Pos;

    // Print all values in a single line
    Serial.print("Enc1Pos: ");
    Serial.print(enc1Pos);
    Serial.print(" | Enc2Pos: ");
    Serial.print(enc2Pos);
    Serial.print(" | Diff: ");
    Serial.print(diff);
    Serial.print(" | Motor1Speed: ");
    Serial.print(motor1Speed);
    Serial.print(" | Motor2Speed: ");
    Serial.println(motor2Speed);

    int bothWheels_flag_count = 0;
    int leftWheels_flag_count = 0;
    int rightWheels_flag_count = 0;

    // Consolidate flag counting into a single loop
    for (int i = 0; i < 8; i++) {
        if (bothWheelsFlags[i]) bothWheels_flag_count++;
        if (leftWheelFlags[i]) leftWheels_flag_count++;
        if (rightWheelFlags[i]) rightWheels_flag_count++;
    }

    // Increment motor speeds if the flag count conditions are met
    if (bothWheels_flag_count > 4) {
        motor1Speed = 130; // Maintain speed of motor 1
        motor2Speed = 145; // Maintain speed of motor 2
        Serial.println("Inside if (bothWheels_flag_count > 5)");

        // Apply the adjusted speeds to the motors
        ledcWrite(PWM_CHANNEL_1, motor1Speed);
        ledcWrite(PWM_CHANNEL_2, motor2Speed);

        for (int i = 0; i < 8; i++) {
            bothWheelsFlags[i] = false;
        }

        delay(130);
    }

  // Adjust motor speeds based on encoder differences
  if (abs(diff) >= 10) {
    if (diff > 0) {
      motor1Speed = 0; // Slow down motor 1
      motor2Speed = 90; // Maintain speed of motor 2
    } else {
      motor1Speed = 100; // Maintain speed of motor 1
      motor2Speed = 0; // Slow down motor 2
    }
    Serial.println("Correcting motor speeds...");
  } else if (abs(diff) < 40) {
    motor1Speed = 105; // Maintain speed of motor 1
    motor2Speed = 125; // Maintain speed of motor 2
  }
    // Apply the adjusted speeds to the motors
    ledcWrite(PWM_CHANNEL_1, motor1Speed);
    ledcWrite(PWM_CHANNEL_2, motor2Speed);
}




void rotateMotor1ToRight() {
    int leftWheels_flag_count = 0;


  for (int i = 0; i < 8; i++) {
      leftWheelFlags[i] = false;
      rightWheelFlags[i] = false;
      bothWheelsFlags[i] = false;
    }
    // Ensure motor 2 is not moving
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, LOW);

    motor1Speed = 100;
    motor2Speed = 0;

    ledcWrite(PWM_CHANNEL_2, motor2Speed);
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    ledcWrite(PWM_CHANNEL_1, motor1Speed);

    // Reset encoder values
    portENTER_CRITICAL(&mux);
    encoder1Pos = 0;
    portEXIT_CRITICAL(&mux);

    Serial.print("Initial Motor 1 Speed: ");
    Serial.println(motor1Speed);
    Serial.print("Initial Motor 2 Speed: ");
    Serial.println(motor2Speed);

    while (true) {
        portENTER_CRITICAL(&mux);
        int enc1Pos = abs(encoder1Pos);
        portEXIT_CRITICAL(&mux);

        leftWheels_flag_count = 0;
        for (int i = 0; i < 8; i++) {
            if (leftWheelFlags[i]) {
                leftWheels_flag_count++;
            }
        }

        Serial.print("Encoder Position: ");
        Serial.println(enc1Pos);
        Serial.print("Left Wheels Flag Count: ");
        Serial.println(leftWheels_flag_count);

        if (leftWheels_flag_count > 4 && motor1Speed != 0 && motor2Speed == 0) {
            Serial.println("Inside if (leftWheels_flag_count > 4)");

            motor1Speed = 145; // Increase speed of motor 1
            ledcWrite(PWM_CHANNEL_1, motor1Speed);

            Serial.print("Adjusted Motor 1 Speed: ");
            Serial.println(motor1Speed);

            for (int i = 0; i < 8; i++) {
                leftWheelFlags[i] = false;
            }

            delay(200);
        }

        if (enc1Pos < 970) {
            // Move motor 1
            digitalWrite(MOTOR1_IN1, HIGH);
            digitalWrite(MOTOR1_IN2, LOW);
            ledcWrite(PWM_CHANNEL_1, motor1Speed);  // Full speed
            Serial.println("Motor 1 is moving...");
        } else {
            // Stop motor 1
            digitalWrite(MOTOR1_IN1, LOW);
            digitalWrite(MOTOR1_IN2, LOW);
            ledcWrite(PWM_CHANNEL_1, 0);

            Serial.println("Motor 1 stopped.");

            // Wait for 0.5 seconds
            delay(500);

            // Reset encoder positions after the delay
            portENTER_CRITICAL(&mux);
            encoder1Pos = 0;
            portEXIT_CRITICAL(&mux);

            // Exit the loop after the rotation is done and encoders are reset
            break;
        }
        delay(10);
    }
}

void rotateMotor2ToLeft() {
  
    for (int i = 0; i < 8; i++) {
      leftWheelFlags[i] = false;
      rightWheelFlags[i] = false;
      bothWheelsFlags[i] = false;
    }
    int rightWheels_flag_count = 0;

    // Ensure motor 1 is not moving
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);

    motor1Speed = 0;
    motor2Speed = 125;

    ledcWrite(PWM_CHANNEL_1, motor1Speed); // Assuming motor 1 is connected to PWM channel 1
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    ledcWrite(PWM_CHANNEL_2, motor2Speed); // Assuming motor 2 is connected to PWM channel 2

    // Reset encoder values
    portENTER_CRITICAL(&mux);
    encoder2Pos = 0;
    portEXIT_CRITICAL(&mux);

    while (true) {
        portENTER_CRITICAL(&mux);
        int enc2Pos = abs(encoder2Pos);
        portEXIT_CRITICAL(&mux);

        rightWheels_flag_count = 0;
        for (int i = 0; i < 8; i++) {
            if (rightWheelFlags[i]) {
                rightWheels_flag_count++;
            }
        }

        if (rightWheels_flag_count > 4 && motor1Speed == 0 && motor2Speed != 0) {
            Serial.println("Inside if (rightWheels_flag_count > 5)");

            motor2Speed = 145; // Maintain speed of motor 2
            ledcWrite(PWM_CHANNEL_2, motor2Speed);

            for (int i = 0; i < 8; i++) {
                rightWheelFlags[i] = false;
            }

            delay(200);
        }

        if (enc2Pos < 1100) {
            // Move motor 2
            digitalWrite(MOTOR2_IN3, HIGH);
            digitalWrite(MOTOR2_IN4, LOW);
            ledcWrite(PWM_CHANNEL_2, motor2Speed);  // Full speed

            Serial.println("Motor 2 is moving...");
        } else {
            // Stop motor 2
            digitalWrite(MOTOR2_IN3, LOW);
            digitalWrite(MOTOR2_IN4, LOW);
            ledcWrite(PWM_CHANNEL_2, 0);

            Serial.println("Motor 2 stopped.");

            // Wait for 0.5 seconds
            delay(500);

            // Reset encoder positions after the delay
            portENTER_CRITICAL(&mux);
            encoder2Pos = 0;
            portEXIT_CRITICAL(&mux);

            // Exit the loop after the rotation is done and encoders are reset
            break;
        }
     
      delay(10);   
    }
}

void turnAround() {

    for (int i = 0; i < 8; i++) {
      leftWheelFlags[i] = false;
      rightWheelFlags[i] = false;
      bothWheelsFlags[i] = false;
    }
    int rightWheels_flag_count = 0;
    int leftWheels_flag_count = 0;

    // Set speeds for both motors
    motor1Speed = 110;
    motor2Speed = 115;

    // Initialize both motors to move in opposite directions
    digitalWrite(MOTOR1_IN1,  LOW );
    digitalWrite(MOTOR1_IN2,  HIGH );
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);

    ledcWrite(PWM_CHANNEL_1, motor1Speed); // Assuming motor 1 is connected to PWM channel 1
    ledcWrite(PWM_CHANNEL_2, motor2Speed); // Assuming motor 2 is connected to PWM channel 2

    // Reset encoder values
    portENTER_CRITICAL(&mux);
    encoder1Pos = 0;
    encoder2Pos = 0;
    portEXIT_CRITICAL(&mux);

    while (true) {
        portENTER_CRITICAL(&mux);
        int enc1Pos = abs(encoder1Pos);
        int enc2Pos = abs(encoder2Pos);
        portEXIT_CRITICAL(&mux);

        leftWheels_flag_count = 0;
        rightWheels_flag_count = 0;

        for (int i = 0; i < 8; i++) {
            if (leftWheelFlags[i]) {
                leftWheels_flag_count++;
            }
            if (rightWheelFlags[i]) {
                rightWheels_flag_count++;
            }
        }

        if (leftWheels_flag_count > 4 && motor1Speed != 0 && motor2Speed != 0) {
            motor1Speed = 145;
            ledcWrite(PWM_CHANNEL_1, motor1Speed);

            for (int i = 0; i < 8; i++) {
                leftWheelFlags[i] = false;
            }

            delay(100);
        }

        if (rightWheels_flag_count > 4 && motor1Speed != 0 && motor2Speed != 0) {
            motor2Speed = 145;
            ledcWrite(PWM_CHANNEL_2, motor2Speed);

            for (int i = 0; i < 8; i++) {
                rightWheelFlags[i] = false;
            }

            delay(100);
        }

        if (enc2Pos >= 1120) {
            // Stop both motors
            digitalWrite(MOTOR1_IN1, LOW);
            digitalWrite(MOTOR1_IN2, LOW);
            digitalWrite(MOTOR2_IN3, LOW);
            digitalWrite(MOTOR2_IN4, LOW);

            ledcWrite(PWM_CHANNEL_1, 0);
            ledcWrite(PWM_CHANNEL_2, 0);

            // Wait for 0.5 seconds
            delay(500);

            // Reset encoder positions after the delay
            portENTER_CRITICAL(&mux);
            encoder1Pos = 0;
            encoder2Pos = 0;
            portEXIT_CRITICAL(&mux);

            // Exit the loop after the rotation is done and encoders are reset
            break;
        }

        delay(10);
    }
}



void stopMotors() {
  // Stop both motors

    for (int i = 0; i < 8; i++) {
      leftWheelFlags[i] = false;
      rightWheelFlags[i] = false;
      bothWheelsFlags[i] = false;
    }
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);

      // Reset encoder values
    portENTER_CRITICAL(&mux);
    encoder1Pos = 0;
    encoder2Pos = 0;
    portEXIT_CRITICAL(&mux);

    delay(300);

}