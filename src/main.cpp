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

// Encoder Counters
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

// Wheel and Movement Parameters
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 21.3
#define PULSES_PER_WHEEL_REV (ENCODER_PULSES_PER_REV * GEAR_RATIO)

// Distance Calculation
#define TARGET_DISTANCE_CM 30
#define TARGET_PULSES ((TARGET_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * PULSES_PER_WHEEL_REV)

// 90-degree Turn Calculation
#define WHEEL_DISTANCE_CM 19
#define TURN_PULSES 160  // Calibrated value for a 90-degree turn

// Encoder Interrupt Service Routines
void IRAM_ATTR encoder1ISR() { encoderCount1++; }
void IRAM_ATTR encoder2ISR() { encoderCount2++; }

// Motor Movement Functions
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

// Accurate 90-degree turn function
void turnRobot(bool turnRight) {
    encoderCount1 = 0;
    encoderCount2 = 0;
    Serial.println(turnRight ? "Turning Right 90°" : "Turning Left 90°");

    // Set motor directions
    if (turnRight) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }

    analogWrite(PWMA, 150);
    analogWrite(PWMB, 150);

    // Use a fixed value based on calibration
    while (encoderCount1 < TURN_PULSES || encoderCount2 < TURN_PULSES) {
        Serial.print("Turning - Encoder1: ");
        Serial.print(encoderCount1);
        Serial.print(" Encoder2: ");
        Serial.println(encoderCount2);
        delay(10);
    }

    stopMotors();
    Serial.println("Turn complete");
}

// Straight-line movement with encoder correction
void moveStraight(int speed, int targetPulses) {
    encoderCount1 = 0;
    encoderCount2 = 0;
    moveForward(speed);

    while (encoderCount1 < targetPulses || encoderCount2 < targetPulses) {
        Serial.print("Encoders - Motor1: ");
        Serial.print(encoderCount1);
        Serial.print(" Motor2: ");
        Serial.println(encoderCount2);

        // Balance motors
        if (encoderCount1 > encoderCount2 + 5) {
            analogWrite(PWMA, speed - 5);
            analogWrite(PWMB, speed + 5);
        } else if (encoderCount2 > encoderCount1 + 5) {
            analogWrite(PWMA, speed + 5);
            analogWrite(PWMB, speed - 5);
        } else {
            analogWrite(PWMA, speed);
            analogWrite(PWMB, speed);
        }

        delay(50);
    }

    stopMotors();
    Serial.println("Straight-line movement complete");
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

    pinMode(ENCODER_A1, INPUT_PULLUP);
    pinMode(ENCODER_B1, INPUT_PULLUP);
    pinMode(ENCODER_A2, INPUT_PULLUP);
    pinMode(ENCODER_B2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2ISR, RISING);

    Serial.println("Setup complete");
}

void loop() {
    int forwardCount = 0;

    for (int i = 0; i < 5; i++) {
        moveStraight(80, TARGET_PULSES);
        delay(1000);

        turnRobot(true);
        delay(1000);

        Serial.println("Moving forward until obstacle detected");
        encoderCount1 = 0;
        encoderCount2 = 0;
        moveForward(80);

        while (true) {
            float distance = getDistance();
            Serial.print("Distance: ");
            Serial.println(distance);

            if (distance < 5) {
                stopMotors();
                forwardCount = (encoderCount1 + encoderCount2) / 2;
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
                Serial.print("Received: ");
                Serial.println(message);
            }
            delay(100);
        }

        if (message == "done") {
            moveStraight(-80, forwardCount);
            delay(1000);

            turnRobot(false);
            delay(1000);

            moveStraight(80, TARGET_PULSES);
        }

        delay(1000);
    }
}
