#include "ISensor.h"

void Sensor::begin() {
    Serial.begin(9600);  // Initialize the serial communication
}

int Sensor::readValue() {
    return analogRead(SENSOR_PIN);  // Read the analog value from the sensor
}
