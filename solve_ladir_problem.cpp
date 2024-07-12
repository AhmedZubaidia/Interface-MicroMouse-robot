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
#define SHT_LOX2 23

// Sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data structures
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;


// Define PWM pins for motor speed control
#define MOTOR1_PWM 5
#define MOTOR2_PWM 32

#define MOTOR1_IN1 19
#define MOTOR1_IN2 18
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

#define IR_PIN1 33
#define IR_PIN2 27

volatile bool ir2Low = false;  // Flag for IR2 being low

unsigned long previousMillis = 0;
const long interval = 10;  // interval to read sensors (10 milliseconds)


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


// ------------------- Interrupt Service Routines -------------------

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
//leftWheelFlags[i] = false;
     // rightWheelFlags[i] = false;
      //bothWheelsFlags[i] = false;
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

void IRAM_ATTR ir2ISR() {
  portENTER_CRITICAL_ISR(&mux);
  ir2Low = digitalRead(IR_PIN2) == LOW;
  portEXIT_CRITICAL_ISR(&mux);
}


void updateElapsedMillis() {
  globalElapsedMillis++;
}




// ------------------- End of Interrupt Service Routines -------------------


// ------------------- Void setup() -------------------
void setup() {
  Serial.begin(115200);

  // Set encoder pins as inputs
  pinMode(ENCODER1_C1, INPUT);
  pinMode(ENCODER1_C2, INPUT);
  pinMode(ENCODER2_C1, INPUT);
  pinMode(ENCODER2_C2, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_C1), updateEncoder1C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_C2), updateEncoder1C2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_C1), updateEncoder2C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_C2), updateEncoder2C2, CHANGE);

  // Set motor control pins as outputs
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);

  // Initialize motors to off
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);

  // Set IR sensor pin as input
  pinMode(IR_PIN2, INPUT);

  // Attach interrupt for IR sensor
  attachInterrupt(digitalPinToInterrupt(IR_PIN2), ir2ISR, CHANGE);

  // Read speeds from EEPROM
  EEPROM.get(EEPROM_ADDR_MOTOR1_SPEED, motor1Speed);
  EEPROM.get(EEPROM_ADDR_MOTOR2_SPEED, motor2Speed);

  // Ensure speeds are within valid range
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  analogWrite(MOTOR1_PWM, motor1Speed);
  analogWrite(MOTOR2_PWM, motor2Speed);

  // Initialize shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins initialized..."));

  // Put both sensors in reset mode
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both sensors in reset mode..."));

  // Start the sensors
  setID();

  // Attach other tickers and timers as needed
  timeTicker.attach(0.001, updateElapsedMillis);  // Increment globalElapsedMillis every 1 ms

  timer = timerBegin(0, 80, true);              // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);  // Attach ISR to timer
  timerAlarmWrite(timer, 1000, true);           // Set alarm to 1 ms (80 MHz / 80 = 1 MHz, so 1000 counts = 1 ms)
  timerAlarmEnable(timer);                      // Enable the alarm
}


void incrementElapsedMillis() {
  elapsedMillis++;
}

// ------------------- End of void setup() -------------------



//--------------------- Void loop() -------------------

void loop() {

    readSensorsAndCheckConditions();
  forward_with_correction(motor1Speed, motor2Speed);
  readDualSensors(); 



}



// ------------------- End of void loop() -------------------


// ------------------- Helper Functions -------------------

void setID() {
  // All sensors in reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // Activate LOX1
  digitalWrite(SHT_LOX1, HIGH);
  delay(50); // wait for the sensor to start

  // Initialize LOX1
  Serial.println(F("Initializing LOX1"));
  if (!lox1.begin(LOX1_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    // Consider retrying initialization or handling error gracefully
    for (int i = 0; i < 3; i++) {
      delay(100);
      if (lox1.begin(LOX1_ADDRESS, &Wire)) {
        Serial.println(F("Successfully initialized LOX1 on retry"));
        break;
      }
      if (i == 2) {
        Serial.println(F("Failed to initialize LOX1 after retries"));
        return; // Exit the function to avoid blocking the rest of the program
      }
    }
  }
  delay(10);

  // Activate LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);

  // Initialize LOX2
  Serial.println(F("Initializing LOX2"));
  if (!lox2.begin(LOX2_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    // Consider retrying initialization or handling error gracefully
    for (int i = 0; i < 3; i++) {
      delay(100);
      if (lox2.begin(LOX2_ADDRESS, &Wire)) {
        Serial.println(F("Successfully initialized LOX2 on retry"));
        break;
      }
      if (i == 2) {
        Serial.println(F("Failed to initialize LOX2 after retries"));
        return; // Exit the function to avoid blocking the rest of the program
      }
    }
  }
}


void readDualSensors() {
  // Perform ranging tests
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // Print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4) {  // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // Print sensor two reading
  Serial.print(F("2: "));
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}



void readSensorsAndCheckConditions() {
  portENTER_CRITICAL(&mux);
  bool ir2 = ir2Low; // Right sensor
  portEXIT_CRITICAL(&mux);

  // Read distance from the sensors
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  // Check if the readings are out of range and assign a large value if they are
  int distance1 = (measure1.RangeStatus == 4) ? 1000 : measure1.RangeMilliMeter; // lidar  right 
  int distance2 = (measure2.RangeStatus == 4) ? 1000 : measure2.RangeMilliMeter; //  lidar left

  Serial.print("IR2 (Right): ");
  Serial.print(ir2);
  Serial.print(" Distance1: ");
  Serial.print(distance1);
  Serial.print(" Distance2: ");
  Serial.print(distance2);
  Serial.println() ;
  if (ir2) {  // If the right sensor detects an obstacle in front
    Serial.println("Right sensor detects an obstacle. Stopping all motors.");
    stopAllMotors();

    if (distance1 > 80 && distance2 < 80 ) {   // go right if left blocked and right open 
      rotateMotor1ToRight();
      rest_moveForward();
    } else if (distance2 > 80 && distance1 < 80) {  // go left if right blocked and left open
      rotateMotor2ToLeft();
      rest_moveForward();
    } else if (distance1 > 80 && distance2 > 80) {  // If both sensors have free space, go right
      rotateMotor1ToRight();
      rest_moveForward();
    } else if (distance1 < 80 && distance2 < 80) {  // If both sensors are blocked, turn around
      Serial.println("Turning around.");
      turnAround();
      rest_moveForward();
    }
  }
}


void setMotorSpeed(int motor, int speed) {
    if (motor == 1) {
        if (speed == 0) {
            digitalWrite(MOTOR1_IN1, LOW);
            digitalWrite(MOTOR1_IN2, LOW);
        } else {
            digitalWrite(MOTOR1_IN1, HIGH);
            digitalWrite(MOTOR1_IN2, LOW);
            analogWrite(MOTOR1_PWM, speed);
        }
    } else if (motor == 2) {
        if (speed == 0) {
            digitalWrite(MOTOR2_IN3, LOW);
            digitalWrite(MOTOR2_IN4, LOW);
        } else {
            digitalWrite(MOTOR2_IN3, HIGH);
            digitalWrite(MOTOR2_IN4, LOW);
            analogWrite(MOTOR2_PWM, speed);
        }
    }
}

void stopAllMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
}



void rest_moveForward() {


  portENTER_CRITICAL(&mux);
  encoder1Pos = 0;
  encoder2Pos = 0;
  portEXIT_CRITICAL(&mux);

  motor1Speed = 100;
  motor2Speed = 100;
  int new_motor1Speed;
  int new_motor2Speed;

  analogWrite(MOTOR1_PWM, motor1Speed);
  analogWrite(MOTOR2_PWM, motor2Speed);

  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
}

void forward_with_correction(int &motor1Speed, int &motor2Speed) {
    int enc1Pos, enc2Pos;
    int diff;

    portENTER_CRITICAL(&mux);
    enc1Pos = encoder1Pos;
    enc2Pos = encoder2Pos;
    portEXIT_CRITICAL(&mux);

    diff = enc1Pos - enc2Pos;

    int bothWheels_flag_count = 0;
    int leftWheels_flag_count = 0;
    int rightWheels_flag_count = 0;

    // Consolidate flag counting into a single loop
    for (int i = 0; i < 8; i++) {
        if (bothWheelsFlags[i]) bothWheels_flag_count++;
        if (leftWheelFlags[i]) leftWheels_flag_count++;
        if (rightWheelFlags[i]) rightWheels_flag_count++;
    }

    // Print debug information
    Serial.print("Enc1: "); Serial.print(enc1Pos); Serial.print(", Enc2: "); Serial.print(enc2Pos);
    Serial.print(", Diff: "); Serial.print(diff); Serial.print(", Motor1Speed: "); Serial.print(motor1Speed);
    Serial.print(", Motor2Speed: "); Serial.print(motor2Speed);
    Serial.print(", BothWheels: "); Serial.print(bothWheels_flag_count);
    Serial.print(", LeftWheels: "); Serial.print(leftWheels_flag_count);
    Serial.print(", RightWheels: "); Serial.println(rightWheels_flag_count);

    // Increment motor speeds if the flag count conditions are met
    if (bothWheels_flag_count > 4) {
        motor1Speed = 135; // Maintain speed of motor 1
        motor2Speed = 165; // Maintain speed of motor 2
        Serial.println("Inside if (bothWheels_flag_count > 4)");
        // Apply the adjusted speeds to the motors
        analogWrite(MOTOR1_PWM, motor1Speed);
        analogWrite(MOTOR2_PWM, motor2Speed);

        for (int i = 0; i < 8; i++) {
            bothWheelsFlags[i] = false;
        }

        delay(130);
    }

    // Adjust motor speeds based on encoder differences
    if (abs(diff) >= 10) {
        if (diff > 0) {
            motor1Speed = 0; // Slow down motor 1
            motor2Speed = 100; // Maintain speed of motor 2
        } else {
            motor1Speed = 95; // Maintain speed of motor 1
            motor2Speed = 0; // Slow down motor 2
        }
        Serial.println("Correcting motor speeds...");
    } else if (abs(diff) < 40) {
        motor1Speed = 95; // Maintain speed of motor 1
        motor2Speed = 100; // Maintain speed of motor 2
    }
    // Apply the adjusted speeds to the motors
    analogWrite(MOTOR1_PWM, motor1Speed);
    analogWrite(MOTOR2_PWM, motor2Speed);
}


// ------------------- End of Helper Functions -------------------




// ------------------- Motor rotation functions -------------------

void rotateMotor2ToLeft() {
  
    int rightWheels_flag_count = 0;

    // Ensure motor 1 is not moving
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);

    motor1Speed = 0;
    motor2Speed = 125;

    analogWrite(MOTOR1_PWM, motor1Speed);
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    analogWrite(MOTOR2_PWM, motor2Speed);

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
            analogWrite(MOTOR2_PWM, motor2Speed);

            for (int i = 0; i < 8; i++) {
                rightWheelFlags[i] = false;
            }

            delay(200);
        }

        if (enc2Pos < 1100) {
            // Move motor 2
            digitalWrite(MOTOR2_IN3, HIGH);
            digitalWrite(MOTOR2_IN4, LOW);
            analogWrite(MOTOR2_PWM, motor2Speed);  // Full speed


        } else {
            // Stop motor 2
            digitalWrite(MOTOR2_IN3, LOW);
            digitalWrite(MOTOR2_IN4, LOW);
            analogWrite(MOTOR2_PWM, 0);

            // Wait for 10 seconds
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

void rotateMotor1ToRight() {
    int leftWheels_flag_count = 0;

    // Ensure motor 2 is not moving
    setMotorSpeed(2, 0);

    motor1Speed = 125;
    setMotorSpeed(1, motor1Speed);

    // Reset encoder values
    portENTER_CRITICAL(&mux);
    encoder1Pos = 0;
    portEXIT_CRITICAL(&mux);

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

        if (leftWheels_flag_count > 4 && motor1Speed != 0) {
            motor1Speed = 145; // Increase speed of motor 1
            setMotorSpeed(1, motor1Speed);

            for (int i = 0; i < 8; i++) {
                leftWheelFlags[i] = false;
            }

            delay(200);
        }

        if (enc1Pos < 1080) {
            // Move motor 1
            setMotorSpeed(1, motor1Speed);
            Serial.println("Motor 1 is moving...");
        } else {
            // Stop motor 1
            setMotorSpeed(1, 0);

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

void turnAround() {
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

    analogWrite(MOTOR1_PWM, motor1Speed);
    analogWrite(MOTOR2_PWM, motor2Speed);

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
            analogWrite(MOTOR1_PWM, motor1Speed);

            for (int i = 0; i < 8; i++) {
                leftWheelFlags[i] = false;
            }

            delay(100);
        }

        if (rightWheels_flag_count > 4 && motor1Speed != 0 && motor2Speed != 0) {
            motor2Speed = 145;
            analogWrite(MOTOR2_PWM, motor2Speed);

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

            analogWrite(MOTOR1_PWM, 0);
            analogWrite(MOTOR2_PWM, 0);

            // Wait for 10 seconds
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


// ------------------- End of Motor rotation functions -------------------