#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include "Arduino.h"
#include <Ticker.h>  // Library for managing timers


// Define PWM pins for motor speed control
#define MOTOR1_PWM 5
#define MOTOR2_PWM 32

// Define motor control pins
#define MOTOR1_IN1 22
#define MOTOR1_IN2 23
#define MOTOR2_IN3 25
#define MOTOR2_IN4 26

// Define encoder pins
#define ENCODER1_C1 34
#define ENCODER1_C2 35
#define ENCODER2_C1 21
#define ENCODER2_C2 19

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

volatile bool ir1Low = false;  // Flag for IR1 being low
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


void IRAM_ATTR ir1ISR() {
  portENTER_CRITICAL_ISR(&mux);
  ir1Low = digitalRead(IR_PIN1) == LOW;
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

    // Set IR sensor pins as inputs
  pinMode(IR_PIN1, INPUT);
  pinMode(IR_PIN2, INPUT);

   // Attach interrupts for IR sensors
  attachInterrupt(digitalPinToInterrupt(IR_PIN1), ir1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_PIN2), ir2ISR, CHANGE);

  // Read speeds from EEPROM
  EEPROM.get(EEPROM_ADDR_MOTOR1_SPEED, motor1Speed);
  EEPROM.get(EEPROM_ADDR_MOTOR2_SPEED, motor2Speed);

  // Ensure speeds are within valid range
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  analogWrite(MOTOR1_PWM, motor1Speed);
  analogWrite(MOTOR2_PWM, motor2Speed);

  // Initialize VL53L0X sensor
  Wire.begin(4, 16);  // SDA = GPIO 16, SCL = GPIO 4
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  // encoderTicker.attach(0.5, checkEncoderDifference);
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


    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readSensorsAndCheckConditions();
  }
  

  portENTER_CRITICAL(&mux);
  int enc1Pos = encoder1Pos;
  int enc2Pos = encoder2Pos;
  portEXIT_CRITICAL(&mux);

  forward_with_correction(motor1Speed, motor2Speed);

  // Print the left wheel flags
  Serial.print("Left Wheel Flags: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(leftWheelFlags[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println();

  // Print the right wheel flags
  Serial.print("Right Wheel Flags: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(rightWheelFlags[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println();

  // Print the both wheels flags
  Serial.print("Both Wheels Flags: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(bothWheelsFlags[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println();


  // Print encoder values
  Serial.print("Encoder 1 Position: ");
  Serial.print(enc1Pos);
  Serial.print("  Encoder 2 Position: ");
  Serial.print(enc2Pos);
  Serial.print("  Motor 1 Speed: ");
  Serial.print(motor1Speed);
  Serial.print("  Motor 2 Speed: ");
  Serial.print(motor2Speed);
  Serial.print("  Distance: ");

  delay(100);  // Adjust delay for smoother control
}



// ------------------- End of void loop() -------------------


// ------------------- Helper Functions -------------------

void readSensorsAndCheckConditions() {
  portENTER_CRITICAL(&mux);
  bool ir1 = ir1Low;
  bool ir2 = ir2Low;
  portEXIT_CRITICAL(&mux);

  // Read distance from the sensor
  int distance = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.println(" TIMEOUT");
    return;
  }

  if (ir2) {  // Only proceed with rotation if IR2 is low
    stopAllMotors();

    if (distance > 80) {
      if (ir1) {
        rotateMotor2ToLeft();
        rest_moveForward();
      } else {
        rotateMotor1ToRight();
        rest_moveForward();

      }
    }
  }
}

void stopAllMotors() {
  // Stop motor 1
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_PWM, 0);

  // Stop motor 2
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_PWM, 0);
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

    // Increment motor speeds if the flag count conditions are met
    if (bothWheels_flag_count > 4) {
         motor1Speed = 135; // Maintain speed of motor 1
         motor2Speed = 165; // Maintain speed of motor 2
        Serial.println("Inside if (bothWheels_flag_count > 5)");
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
      motor1Speed = 10; // Slow down motor 1
      motor2Speed = 115; // Maintain speed of motor 2
    } else {
      motor1Speed = 85; // Maintain speed of motor 1
      motor2Speed = 0; // Slow down motor 2
    }
    Serial.println("Correcting motor speeds...");
  } else if (abs(diff) < 40) {
    motor1Speed = 85; // Maintain speed of motor 1
    motor2Speed = 115; // Maintain speed of motor 2
  }
    // Apply the adjusted speeds to the motors
    analogWrite(MOTOR1_PWM, motor1Speed);
    analogWrite(MOTOR2_PWM, motor2Speed);

    
}



// ------------------- End of Helper Functions -------------------




// ------------------- Motor rotation functions -------------------


void rotateMotor2ToLeft() {

  //stopCheckingEncoderDifference();


  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);

  

  motor1Speed = 0;
  motor2Speed = 110;
  analogWrite(MOTOR1_PWM, motor1Speed);

  digitalWrite(MOTOR2_IN3, HIGH);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR2_PWM, motor2Speed);

  // Reset encoder values
  portENTER_CRITICAL(&mux);
  encoder2Pos = 0;
  portEXIT_CRITICAL(&mux);

  // startCheckingEncoderDifference();

    int bothWheels_flag_count = 0;
    int leftWheels_flag_count = 0;
    int rightWheels_flag_count = 0;

  while (true) {
    portENTER_CRITICAL(&mux);
    int enc2Pos = encoder2Pos;
    portEXIT_CRITICAL(&mux);


    for (int i = 0; i < 8; i++) {
        if (rightWheelFlags[i]) {
            rightWheels_flag_count++;
        }
    }

    if(rightWheels_flag_count > 5 && motor1Speed == 0 && motor2Speed != 0) {
        motorIncrement2 = 30;
        Serial.println("Inside if (rightWheels_flag_count > 5)");

        for (int i = 0; i < 8; i++) {
            rightWheelFlags[i] = false;
        }
    }

    if (enc2Pos < 1086) {
      // Move motor 2
      digitalWrite(MOTOR2_IN3, HIGH);
      digitalWrite(MOTOR2_IN4, LOW);

       motor2Speed = motor2Speed + motorIncrement2;
      analogWrite(MOTOR2_PWM, motor2Speed );  // Full speed
    } else {
      // Stop motor 2
      digitalWrite(MOTOR2_IN3, LOW);
      digitalWrite(MOTOR2_IN4, LOW);

      motor2Speed = 0;
      analogWrite(MOTOR2_PWM, motor2Speed);

      // Print final encoder value for debugging
      Serial.print("Encoder 2 Position before reset: ");
      Serial.println(enc2Pos);

      // Wait for 10 seconds
      delay(500);

      // Reset encoder positions after the delay
      portENTER_CRITICAL(&mux);
      encoder2Pos = 0;
      portEXIT_CRITICAL(&mux);

      // Print reset message for debugging
      Serial.println("Encoder 2 Position reset.");

      // Exit the loop after the rotation is done and encoders are reset
      break;
    }

    // Print encoder value for debugging
    Serial.print("Encoder 2 Position: ");
    Serial.println(enc2Pos);
    motorIncrement2 =0 ;

    // Small delay to prevent overwhelming the serial output
    delay(100);
  }
}
void rotateMotor1ToRight() {



    int leftWheels_flag_count = 0;
    int motorIncrement1 = 0;

  // Ensure motor 2 is not moving
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);

   motor1Speed = 110;
   motor2Speed = 0 ;

  analogWrite(MOTOR2_PWM, motor2Speed);
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_PWM, motor1Speed);

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

  if(leftWheels_flag_count  > 4 && motor1Speed !=0 && motor2Speed == 0) {
        Serial.println("Inside if (rightWheels_flag_count > 5)");

  
         motor2Speed = 135; // Maintain speed of motor 2
          analogWrite(MOTOR1_PWM, motor1Speed);

            for (int i = 0; i < 8; i++) {
            rightWheelFlags[i] = false;
        }


        delay(200);
    }


    if (enc1Pos < 1086) {
      // Move motor 1
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_PWM, motor1Speed);  // Full speed
    } else {

      
      // Stop motor 1
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_PWM, 0);

      // Print final encoder value for debugging
      Serial.print("Encoder 1 Position before reset: ");
      Serial.println(enc1Pos);

      // Wait for 10 seconds
      delay(500);

      // Reset encoder positions after the delay
      portENTER_CRITICAL(&mux);
      encoder1Pos = 0;
      portEXIT_CRITICAL(&mux);

      // Print reset message for debugging
      Serial.println("Encoder 1 Position reset.");

      // Exit the loop after the rotation is done and encoders are reset
      break;
    }

    // Print encoder value for debugging
    Serial.print("Encoder 1 Position: ");
    Serial.println(enc1Pos);
            motorIncrement1 = 0;


    // Small delay to prevent overwhelming the serial output
    delay(100);
  }
}


void rotateMotor1ToRight_backward() {

  //stopCheckingEncoderDifference();


  // Ensure motor 2 is not moving
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);

  motor1Speed = 100;
  motor2Speed = 0;
  analogWrite(MOTOR2_PWM, motor2Speed);


  // Reset encoder values
  portENTER_CRITICAL(&mux);
  encoder1Pos = 0;
  portEXIT_CRITICAL(&mux);

  //startCheckingEncoderDifference();

  while (true) {
    portENTER_CRITICAL(&mux);
    int enc1Pos = abs(encoder1Pos);
    portEXIT_CRITICAL(&mux);

    if (enc1Pos < 1020) {
      // Move motor 1
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
      analogWrite(MOTOR1_PWM, motor1Speed);  // Full speed
    } else {
      // Stop motor 1
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);

      motor1Speed = 0;

      analogWrite(MOTOR1_PWM, motor1Speed);

      // Print final encoder value for debugging
      Serial.print("Encoder 1 Position before reset: ");
      Serial.println(enc1Pos);

      // Wait for 10 seconds
      delay(500);

      // Reset encoder positions after the delay
      portENTER_CRITICAL(&mux);
      encoder1Pos = 0;
      portEXIT_CRITICAL(&mux);

      // Print reset message for debugging
      Serial.println("Encoder 1 Position reset.");

      // Exit the loop after the rotation is done and encoders are reset
      break;
    }

    // Print encoder value for debugging
    Serial.print("Encoder 1 Position: ");
    Serial.println(enc1Pos);

    // Small delay to prevent overwhelming the serial output
    delay(100);
  }
}
// ------------------- End of Motor rotation functions -------------------