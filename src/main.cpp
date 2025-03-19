#include <Arduino.h>
#include <HardwareSerial.h>
#include "ultrasonic.h"
#include "coloursensor.h"


HardwareSerial mySerial(1);

void setup() {
    Serial.begin(115200);
    setupUltrasonic();
    setupcoloursensor();
    mySerial.begin(9600, SERIAL_8N1, 16, 17); 

}

void loop() {
    String colour = getcolour();
    if (colour == "RED") {
        mySerial.println("RED");
        delay(1000);
    
    } else if (colour == "BLUE") {
        mySerial.println("BLUE");
        delay (500);
    } 
    float distance = getDistance();
    
    if (distance > 0) { 
        if (distance < 10) {
            mySerial.println("Arrived");
        } else {
            mySerial.println("Not arrived");
        }
    }
    delay(1000);  
}




