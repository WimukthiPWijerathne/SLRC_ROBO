#include <Arduino.h>
#include <HardwareSerial.h>
#include "ultrasonic.h"
#include "coloursensor.h"

// Motor Driver Pins
#define AIN1 27
#define AIN2 26
#define PWMA 14
#define BIN1 25
#define BIN2 33
#define PWMB 32

// Encoder Pins
#define ENCODER_A1 13  // Motor 1 Encoder A
#define ENCODER_B1 22  // Motor 1 Encoder B
#define ENCODER_A2 21  // Motor 2 Encoder A
#define ENCODER_B2 23  // Motor 2 Encoder B

// Encoder counters
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

// Target distance in centimeters
#define TARGET_DISTANCE_CM 30
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 21.3
#define PULSES_PER_WHEEL_REV (ENCODER_PULSES_PER_REV * GEAR_RATIO)
#define TARGET_PULSES ((TARGET_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * PULSES_PER_WHEEL_REV)

// Encoder-based 90-degree turn (19 cm wheelbase)
#define WHEEL_DISTANCE_CM 19
// For a 90-degree turn, each wheel needs to travel an arc that is 1/4 of the circumference of the circle with diameter = wheel distance
// Arc length = (PI * wheel_distance) / 4
// Number of wheel rotations = arc length / wheel circumference
// Number of pulses = rotations * pulses per rotation
#define TURN_PULSES 155  // Adjusted value for more accurate 90-degree turn

// Encoder Interrupt Service Routines
void IRAM_ATTR encoder1ISR() {
  encoderCount1++;
}

void IRAM_ATTR encoder2ISR() {
  encoderCount2++;
}

// Motor movement functions
void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void moveBackward(int speed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speed);
}

void stopMotors() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

// Accurate 90-degree right turn using encoder counts
// Motor movement functions with reduced speed
// Adjusted TURN_PULSES based on your encoder readings
#define TURN_PULSES 148  // Reduced from 171 to match observed behavior

// Motor movement functions with reduced speed
void turnRight90Degrees() {
    encoderCount1 = 0;
    encoderCount2 = 0;
    
    Serial.println("Starting right turn");

    // Reduce speed for finer control
    int turnSpeed = 120; // Reduced speed for better precision

    // Motor 1 Forward, Motor 2 Backward (Pivot turn)
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turnSpeed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, turnSpeed);

    // Monitor both encoders
    while (encoderCount1 < TURN_PULSES && encoderCount2 < TURN_PULSES) {
        Serial.print("Turn Right - Encoder1: ");
        Serial.print(encoderCount1);
        Serial.print(" Encoder2: ");
        Serial.println(encoderCount2);
        delay(10);
    }
    
    stopMotors();
    Serial.println("Right turn completed");
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



HardwareSerial mySerial(1);

void setup() {
    Serial.begin(115200);
    setupUltrasonic();
    setupcoloursensor();
    mySerial.begin(9600, SERIAL_8N1, 16, 17);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Encoder pins as input with pullup resistors
    pinMode(ENCODER_A1, INPUT_PULLUP);
    pinMode(ENCODER_B1, INPUT_PULLUP);
    pinMode(ENCODER_A2, INPUT_PULLUP);
    pinMode(ENCODER_B2, INPUT_PULLUP);

    // Attach interrupts to encoder pins
    attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2ISR, RISING);
    
    Serial.println("Setup complete");
    Serial.print("TARGET_PULSES for 30cm: ");
    Serial.println(TARGET_PULSES);
    Serial.print("TURN_PULSES for 90 degrees: ");
    Serial.println(TURN_PULSES);
}

void loop() {
    encoderCount1 = 0;
    encoderCount2 = 0;
    int forwardCount = 0;

    for (int i = 0; i < 5; i++) {
        Serial.println("Moving forward 30cm");
        moveForward(30);
    
        // Monitor both encoders for straight line
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
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
        Serial.println("Stopped after 30cm");
        delay(1000);
        
        Serial.println("Turning right 90 degrees");
        turnRight90Degrees();
        Serial.println("Turn complete");
        delay(1000);

        encoderCount1 = 0;
        encoderCount2 = 0;

        Serial.println("Moving forward until obstacle");
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
                forwardCount = (encoderCount1 + encoderCount2) / 2; // Average of both encoders
                Serial.print("Arrived at obstacle. Forward count: ");
                Serial.println(forwardCount);
                mySerial.println("Arrived");
                break;
            }
            delay(50);
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

            Serial.println("Moving forward 30cm");
            encoderCount1 = 0;
            encoderCount2 = 0;
            moveForward(80);
            while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
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
            Serial.println("Forward movement complete");
        }

        delay(1000);
    }
}
