#include <Arduino.h>
#include <QTRSensors.h>
#include <HardwareSerial.h>
#include "ultrasonic.h"
#include "coloursensor.h"
#include "ISensor.h"




Sensor sensor; 
HardwareSerial mySerial(1);


// Motor Driver Pins
#define AIN1 27
#define AIN2 26
#define PWMA 14
#define BIN1 25
#define BIN2 33
#define PWMB 32

// Encoder Pins
#define ENCODER_A1 13
#define ENCODER_B1 22
#define ENCODER_A2 21
#define ENCODER_B2 23

// Encoder counters
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

// Movement Parameters
#define TARGET_DISTANCE_CM 30
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 21.3
#define PULSES_PER_WHEEL_REV (ENCODER_PULSES_PER_REV * GEAR_RATIO)
#define TARGET_PULSES ((TARGET_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * PULSES_PER_WHEEL_REV)
#define TURN_PULSES 155  // Adjusted for accurate 90-degree turn

// ISR for Encoders
void IRAM_ATTR encoder1ISR() { encoderCount1++; }
void IRAM_ATTR encoder2ISR() { encoderCount2++; }



// Motor Functions
void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void stopMotors() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}
void moveBackward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed);
}
void turnRight90Degrees() {
    encoderCount1 = 0;
    encoderCount2 = 0;
    Serial.println("Turning Right");
    int turnSpeed = 80;
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turnSpeed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, turnSpeed);
    while (encoderCount1 < TURN_PULSES || encoderCount2 < TURN_PULSES) {}
    stopMotors();
    Serial.println("Turn Completed");
}
void turnLeft90Degrees() {
  encoderCount1 = 0;
  encoderCount2 = 0;
  
  Serial.println("Starting left turn");

  // Reduce speed for finer control
  int turnSpeed = 40; // Reduced speed for better precision

  // Motor 1 Backward, Motor 2 Forward (Pivot turn)
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, turnSpeed);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, turnSpeed);

  // Monitor both encoders
  while (encoderCount1 < TURN_PULSES && encoderCount2 < TURN_PULSES) {
      Serial.print("Turn Left - Encoder1: ");
      Serial.print(encoderCount1);
      Serial.print(" Encoder2: ");
      Serial.println(encoderCount2);
      delay(10);
  }
  
  stopMotors();
  Serial.println("Left turn completed");
}

void setup() {
    Serial.begin(9600);
    sensor.begin();
    setupUltrasonic();
    setupcoloursensor();

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(ENCODER_A1, INPUT_PULLUP);
    pinMode(ENCODER_B1, INPUT_PULLUP);
    pinMode(ENCODER_A2, INPUT_PULLUP);
    pinMode(ENCODER_B2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2ISR, RISING);

    Serial.println("Setup Complete");
}

void loop() {
  float distance = getDistance(); 
  int sensorValue = sensor.readValue(); 
  Serial.println(sensorValue);

  if (sensorValue > 500){
    moveForward(40);
  }
  else{
    stopMotors();
    delay(1000);
    moveForward(30);
    while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {}
    stopMotors();
    delay(1000);
    turnRight90Degrees();
    stopMotors();
    delay(500);
    encoderCount1 = 0;
    encoderCount2 = 0;
    moveForward(80);

    while (true) {
        float distance = getDistance();
        Serial.print("Distance: ");
        Serial.println(distance);

        // Balance motors for straight line
        if (encoderCount1 > encoderCount2 + 5) {
            analogWrite(PWMA, 75);
            analogWrite(PWMB, 85);
        } else if (encoderCount2 > encoderCount1 + 5) {
            analogWrite(PWMA, 85);
            analogWrite(PWMB, 75);
        } else {
            analogWrite(PWMA, 80);
            analogWrite(PWMB, 80);
        }

        if (distance < 5) {
            stopMotors();
           int forwardCount = (encoderCount1 + encoderCount2) / 2;
            Serial.print("Arrived at obstacle. Forward count: ");
            Serial.println(forwardCount);
            mySerial.println("Arrived");
            break;
        }
        delay(50);
    }
  }
  Serial.println("Waiting for 'done' message");
        String message = "";
        while (message != "done") {
            if (mySerial.available()) {
                message = mySerial.readStringUntil('\n');
                message.trim();
                Serial.print("Received message: ");
                Serial.println(message);
            }
            delay(100);
        }
        if (message == "done") {
        Serial.println("Moving backward");
        encoderCount1 = 0;
        encoderCount2 = 0;
        moveBackward(80);
      
        while (encoderCount1 < forwardCount || encoderCount2 < forwardCount) {
        Serial.print("Encoder1: ");
        Serial.print(encoderCount1);
        Serial.print(" Encoder2: ");
        Serial.println(encoderCount2);
                      
                      // Balance motors if needed
        if (encoderCount1 > encoderCount2 + 5) {
            analogWrite(PWMA, 75);
            analogWrite(PWMB, 85);
            } else if (encoderCount2 > encoderCount1 + 5) {
            analogWrite(PWMA, 85);
            analogWrite(PWMB, 75);
            } else {
            analogWrite(PWMA, 80);
            analogWrite(PWMB, 80);
            }
                      
          delay(50);
         }
          stopMotors();
          Serial.println("Backward movement complete");
      
          Serial.println("Turning left 90 degrees");
          turnLeft90Degrees();
          Serial.println("Turn complete");
          delay(1000);
        
    
   
     
}

    

    //     Serial.println("Moving forward until obstacle");
    //     moveForward(80);
    //     while (true) {
    //         float distance = getDistance();
    //         Serial.print("Distance: ");
    //         Serial.println(distance);
            
    //         // Balance motors for straight line
    //         if (encoderCount1 > encoderCount2 + 5) {
    //             analogWrite(PWMA, 75);
    //             analogWrite(PWMB, 85);
    //         } else if (encoderCount2 > encoderCount1 + 5) {
    //             analogWrite(PWMA, 85);
    //             analogWrite(PWMB, 75);
    //         } else {
    //             analogWrite(PWMA, 80);
    //             analogWrite(PWMB, 80);
    //         }
            
    //         if (distance < 5) { 
    //             stopMotors();
    //             forwardCount = (encoderCount1 + encoderCount2) / 2; // Average of both encoders
    //             Serial.print("Arrived at obstacle. Forward count: ");
    //             Serial.println(forwardCount);
    //             mySerial.println("Arrived");
    //             break;
    //         }
    //         delay(50);
    //     }

    //     
        
    //    

    //         Serial.println("Moving forward 30cm");
    //         encoderCount1 = 0;
    //         encoderCount2 = 0;
    //         moveForward(80);
    //         while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
    //             Serial.print("Encoder1: ");
    //             Serial.print(encoderCount1);
    //             Serial.print(" Encoder2: ");
    //             Serial.println(encoderCount2);
                
    //             // Balance motors if needed
    //             if (encoderCount1 > encoderCount2 + 5) {
    //                 analogWrite(PWMA, 75);
    //                 analogWrite(PWMB, 85);
    //             } else if (encoderCount2 > encoderCount1 + 5) {
    //                 analogWrite(PWMA, 85);
    //                 analogWrite(PWMB, 75);
    //             } else {
    //                 analogWrite(PWMA, 80);
    //                 analogWrite(PWMB, 80);
    //             }
                
    //             delay(50);
    //         }
    //         stopMotors();
    //         Serial.println("Forward movement complete");
    //     }

    //     delay(1000);
    // }



// #include <Arduino.h>

//          // Emitter pin for the IR sensors

// // Instantiate the IRSensorArray object
// 

// // Motor driver pins
// #define AIN1 27
// #define AIN2 26
// #define PWMA 14
// #define BIN1 25
// #define BIN2 33
// #define PWMB 32

// // Motor movement functions
// void moveForward(int speed) {
//     digitalWrite(AIN1, HIGH);
//     digitalWrite(AIN2, LOW);
//     analogWrite(PWMA, speed);
//     digitalWrite(BIN1, HIGH);
//     digitalWrite(BIN2, LOW);
//     analogWrite(PWMB, speed);
// }

// void stopMotors() {
//     analogWrite(PWMA, 0);
//     analogWrite(PWMB, 0);
// }

// void turnRight90Degrees() {
//     // Implement a 90-degree turn logic using encoders or timed turn
//     // Just a simple placeholder for turning right
//     // Rotate for a fixed time or using encoder counts
//     digitalWrite(AIN1, HIGH);
//     digitalWrite(AIN2, LOW);
//     analogWrite(PWMA, 80);
//     digitalWrite(BIN1, LOW);
//     digitalWrite(BIN2, HIGH);
//     analogWrite(PWMB, 80);
//     delay(1000);  // Adjust time for 90-degree turn
//     stopMotors();
// }

// void setup() {
//     Serial.begin(9600);
//       // Calibrate the sensor array

//     pinMode(AIN1, OUTPUT);
//     pinMode(AIN2, OUTPUT);
//     pinMode(PWMA, OUTPUT);
//     pinMode(BIN1, OUTPUT);
//     pinMode(BIN2, OUTPUT);
//     pinMode(PWMB, OUTPUT);
// }

// void loop() {
//     // Read the line position from the sensor array
//        // Wait for 1000 milliseconds (1 second)

//         // Move forward 15 cm
//         // Assuming a function or method that moves forward by 15 cm
//         // You might need to adjust this to your encoder-based approach
//         Serial.println("Moving forward 15cm");
//         moveForward(30);  // Adjust speed as needed
//         delay(1000);      // Adjust time to achieve 15cm

//         stopMotors();  // Stop motors after moving 15 cm
//         Serial.println("Completed 15cm");

//         // Turn 90 degrees
//         turnRight90Degrees();
//         Serial.println("Completed 90-degree turn");

//         delay(1000);  // Delay before repeating the process
//     }
