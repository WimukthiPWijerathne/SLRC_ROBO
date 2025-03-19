#include <Arduino.h>
#include "ultrasonic.h"

#define TRIG_PIN 4
#define ECHO_PIN 15

void setupUltrasonic() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
}

float getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(15);  // Increased pulse width
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    if (duration == 0) {
        Serial.println("Error: No Echo received!");
        return -1;  // Return -1 if no valid reading
    } 
    float distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
}
