#include <Arduino.h>

#define TRIG_PIN 15
#define ECHO_PIN 4
#define TEMPERATURE 25 // Ambient temperature in Celsius
#define SAMPLES 5      // Number of samples to average
#define MAX_DISTANCE 400 // Maximum distance in cm
#define MIN_DISTANCE 2   // Minimum distance in cm
#define TIMEOUT 38000 // Timeout for pulseIn (38ms ≈ 6.5m)





float distanceArray[SAMPLES];
int arrayIndex = 0;
float filteredDistance = 0;
float ALPHA = 0.2; // Exponential filter factor (0-1)

void setupUltrasonic() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    for (int i = 0; i < SAMPLES; i++) {
        distanceArray[i] = 0;
    }
}

float getDistance() {
    float rawDistances[SAMPLES];
    int validReadings = 0;
    float speedOfSound = 331.3 + (0.606 * TEMPERATURE); // Speed in m/s

    for (int i = 0; i < SAMPLES; i++) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);

        if (duration > 0) {
            float distance = (duration * speedOfSound / 10000) / 2;
            if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
                rawDistances[validReadings++] = distance; 
            }
        }
        delay(10);
    }

    if (validReadings == 0) {
        Serial.println("Error: No valid readings!");
        return -1;
    }

    // **Using Median Filter**
    float medianDistance;
    if (validReadings == 1) {
        medianDistance = rawDistances[0]; // Only one valid reading
    } else {
        // Sort for median
        for (int i = 0; i < validReadings - 1; i++) {
            for (int j = 0; j < validReadings - i - 1; j++) {
                if (rawDistances[j] > rawDistances[j + 1]) {
                    float temp = rawDistances[j];
                    rawDistances[j] = rawDistances[j + 1];
                    rawDistances[j + 1] = temp;
                }
            }
        }
        medianDistance = rawDistances[validReadings / 2]; // Median value
    }

    // **Exponential Filter**
    filteredDistance = (filteredDistance == 0) ? medianDistance : (ALPHA * medianDistance + (1 - ALPHA) * filteredDistance);

    Serial.print("Distance: ");
    Serial.print(filteredDistance);
    Serial.println(" cm");

    return filteredDistance;
}
