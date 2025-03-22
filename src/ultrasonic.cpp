#include <Arduino.h>
#include "ultrasonic.h"

#define TRIG_PIN 15
#define ECHO_PIN 4
#define TEMPERATURE 25 // Ambient temperature in Celsius
#define SAMPLES 5      // Number of samples to average
#define MAX_DISTANCE 400 // Maximum distance in cm
#define MIN_DISTANCE 2   // Minimum distance in cm

// Variables for moving average filter
float distanceArray[SAMPLES];
int arrayIndex = 0;

// Variables for exponential filter
float filteredDistance = 0;
float ALPHA = 0.2; // Smoothing factor (0-1), lower = smoother

void setupUltrasonic() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Initialize the array with zeros
    for (int i = 0; i < SAMPLES; i++) {
        distanceArray[i] = 0;
    }
}

float getDistance() {
    // Take multiple readings and discard outliers
    float rawDistances[SAMPLES];
    int validReadings = 0;
    
    // Calculate speed of sound based on temperature
    float speedOfSound = 331.3 + (0.606 * TEMPERATURE);
    
    // Take multiple readings
    for (int i = 0; i < SAMPLES; i++) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        
        long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms
        
        if (duration > 0) {
            // Calculate distance with temperature compensation
            float distance = (duration * speedOfSound / 10000) / 2; // in cm
            
            // Only accept readings within valid range
            if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
                rawDistances[i] = distance;
                validReadings++;
            } else {
                rawDistances[i] = -1; // Mark as invalid
            }
        } else {
            rawDistances[i] = -1; // Mark as invalid
        }
        
        delay(10); // Short delay between readings
    }
    
    // If no valid readings, return error
    if (validReadings == 0) {
        Serial.println("Error: No valid readings!");
        return -1;
    }
    
    // Choose your preferred filtering method:
    
    // OPTION 1: Simple average of valid readings
    float sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        if (rawDistances[i] > 0) {
            sum += rawDistances[i];
        }
    }
    float averageDistance = sum / validReadings;
    
    // OPTION 2: Moving average filter
    distanceArray[arrayIndex] = averageDistance;
    arrayIndex = (arrayIndex + 1) % SAMPLES;
    
    float movingAverage = 0;
    for (int i = 0; i < SAMPLES; i++) {
        movingAverage += distanceArray[i];
    }
    movingAverage /= SAMPLES;
    
    // OPTION 3: Exponential filter (low-pass)
    if (filteredDistance == 0) {
        filteredDistance = averageDistance; // Initialize on first reading
    } else {
        filteredDistance = ALPHA * averageDistance + (1 - ALPHA) * filteredDistance;
    }
    
    // OPTION 4: Median filter
    // First, copy valid readings to a new array
    float validDistances[validReadings];
    int validIndex = 0;
    for (int i = 0; i < SAMPLES; i++) {
        if (rawDistances[i] > 0) {
            validDistances[validIndex++] = rawDistances[i];
        }
    }
    
    // Simple bubble sort to find median
    for (int i = 0; i < validReadings - 1; i++) {
        for (int j = 0; j < validReadings - i - 1; j++) {
            if (validDistances[j] > validDistances[j + 1]) {
                float temp = validDistances[j];
                validDistances[j] = validDistances[j + 1];
                validDistances[j + 1] = temp;
            }
        }
    }
    float medianDistance = validDistances[validReadings / 2];
    
    // Choose which filter result to return
    // Uncomment the one you want to use:
    float finalDistance = filteredDistance; // Using exponential filter
    // float finalDistance = movingAverage; // Using moving average
    // float finalDistance = medianDistance; // Using median filter
    // float finalDistance = averageDistance; // Using simple average
    
    Serial.print("Distance: ");
    Serial.print(finalDistance);
    Serial.println(" cm");
    
    return finalDistance;
}
