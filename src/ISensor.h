#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// Pin definition for the sensor
#define SENSOR_PIN 35

class Sensor {
public:
    void begin();                 // Initialize the sensor
    int readValue();              // Read sensor value
};

#endif
