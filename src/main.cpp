#include <Arduino.h>
#include <HardwareSerial.h>
#include "ultrasonic.h"  // Assumed to have getUltrasonicDistance()
#include "coloursensor.h" // Assumed to have getColor()

// Motor Driver Pins
#define AIN1 27
#define AIN2 26
#define PWMA 14
#define BIN1 25
#define BIN2 33
#define PWMB 32

// Encoder Pins
#define LEFT_ENCODER_A 13
#define LEFT_ENCODER_B 21
#define RIGHT_ENCODER_A 22
#define RIGHT_ENCODER_B 23

// Movement Parameters
#define TARGET_DISTANCE_CM 30
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 21.3
#define PULSES_PER_WHEEL_REV (ENCODER_PULSES_PER_REV * GEAR_RATIO)
#define TARGET_PULSES ((TARGET_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * PULSES_PER_WHEEL_REV)
#define TURN_PULSES 165  // Adjustable via calibration

// PID Constants (Tune these)
#define KP 1.0
#define KI 0.05
#define KD 0.1

// Encoder Counters
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// State Machine
enum State { IDLE, MOVING_FORWARD, TURNING_RIGHT, TURNING_LEFT, OBSTACLE_AVOID };
State currentState = IDLE;

// Timing
unsigned long lastUpdate = 0;
const unsigned long INTERVAL = 50;

// Motor Class
class Motor {
public:
    Motor(int in1, int in2, int pwm) : pin1(in1), pin2(in2), pwmPin(pwm) {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(pwmPin, OUTPUT);
    }
    void forward(int speed) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwmPin, speed);
    }
    void backward(int speed) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwmPin, speed);
    }
    void stop() { analogWrite(pwmPin, 0); }
private:
    int pin1, pin2, pwmPin;
};

// Instantiate Motors
Motor leftMotor(AIN1, AIN2, PWMA);
Motor rightMotor(BIN1, BIN2, PWMB);

// Encoder ISRs with Quadrature
void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
}

// PID Control
int baseSpeed = 80;
float error, lastError = 0, integral = 0;

void adjustMotorSpeeds() {
    error = leftEncoderCount - rightEncoderCount; // Difference between encoders
    integral += error;
    float derivative = error - lastError;
    float correction = KP * error + KI * integral + KD * derivative;

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    leftMotor.forward(leftSpeed);
    rightMotor.forward(rightSpeed);

    lastError = error;
}

// Movement Functions
void moveForward(int speed) {
    baseSpeed = speed;
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    currentState = MOVING_FORWARD;
}

void turnRight90Degrees() {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    leftMotor.forward(150);
    rightMotor.backward(150);
    currentState = TURNING_RIGHT;
}

void turnLeft90Degrees() {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    leftMotor.backward(150);
    rightMotor.forward(150);
    currentState = TURNING_LEFT;
}

void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
    currentState = IDLE;
}

// HardwareSerial for communication
HardwareSerial mySerial(1);

void setup() {
    Serial.begin(115200);
    setupUltrasonic();
    setupcoloursensor();
    mySerial.begin(9600, SERIAL_8N1, 16, 17);

    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdate >= INTERVAL) {
        lastUpdate = currentTime;

        // Sensor Integration
        float distance = getDistance(); // From ultrasonic.h
        if (distance < 10.0 && currentState != OBSTACLE_AVOID) {
            stopMotors();
            currentState = OBSTACLE_AVOID;
            Serial.println("Obstacle detected!");
        }

        // State Machine
        switch (currentState) {
            case IDLE:
                Serial.println("Moving forward 30cm");
                moveForward(80);
                break;

            case MOVING_FORWARD:
                adjustMotorSpeeds();
                if (leftEncoderCount >= TARGET_PULSES && rightEncoderCount >= TARGET_PULSES) {
                    stopMotors();
                    Serial.println("Turning right 90 degrees");
                    turnRight90Degrees();
                }
                break;

            case TURNING_RIGHT:
                if (abs(leftEncoderCount) >= TURN_PULSES && abs(rightEncoderCount) >= TURN_PULSES) {
                    stopMotors();
                }
                break;

            case TURNING_LEFT:
                if (abs(leftEncoderCount) >= TURN_PULSES && abs(rightEncoderCount) >= TURN_PULSES) {
                    stopMotors();
                }
                break;

            case OBSTACLE_AVOID:
                turnRight90Degrees();
                break;
        }

        // Encoder Diagnostics
        if (leftEncoderCount == 0 && rightEncoderCount == 0 && currentState != IDLE) {
            Serial.println("Warning: Encoders not responding!");
        }
    }
}