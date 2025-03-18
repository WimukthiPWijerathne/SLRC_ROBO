#include <Arduino.h>
// #include <Wire.h>
// #include <MPU6050.h>

// MPU6050 mpu;

// long previousTime = 0;
// float accAngleX, accAngleY, gyroAngleX, gyroAngleY, angleX, angleY;
// float gyroX, gyroY, gyroZ;
// float elapsedTime;
// float alpha = 0.98; // Complementary filter weight

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();
    
//     mpu.initialize();
    
//     if (!mpu.testConnection()) {
//         Serial.println("MPU6050 connection failed!");
//         while (1);
//     }
    
//     Serial.println("MPU6050 initialized successfully!");
//     delay(1000);
// }

// void loop() {
//     long currentTime = millis();
//     elapsedTime = (currentTime - previousTime) / 1000.0;
//     previousTime = currentTime;
    
//     int16_t ax, ay, az, gx, gy, gz;
//     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     // Convert raw accelerometer data to angles
//     accAngleX = atan2(ay, az) * 180 / M_PI;
//     accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

//     // Convert raw gyro data to angular velocity (dps)
//     gyroX = gx / 131.0;
//     gyroY = gy / 131.0;
//     gyroZ = gz / 131.0;

//     // Integrate gyro data to get angles
//     gyroAngleX += gyroX * elapsedTime;
//     gyroAngleY += gyroY * elapsedTime;

//     // Complementary filter to combine accelerometer and gyroscope values
//     angleX = alpha * (angleX + gyroX * elapsedTime) + (1 - alpha) * accAngleX;
//     angleY = alpha * (angleY + gyroY * elapsedTime) + (1 - alpha) * accAngleY;

//     // Display angles in Serial Monitor
//     Serial.print("Roll (X): "); Serial.print(angleX); Serial.print("°  ");
//     Serial.print("Pitch (Y): "); Serial.print(angleY); Serial.println("°");

//     delay(50);
// }


#include <HardwareSerial.h>
#define TRIG_PIN 4
#define ECHO_PIN 15

HardwareSerial mySerial(1);  // Use UART1

void setup() {
    Serial.begin(115200);
   
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    mySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
}
void send () {
    
        mySerial.println("arrived");
        delay(3000);
    
}

float distance () {
    
  long duration;
  float distance;

  // Trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);  // Increased pulse width
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo duration
  duration = pulseIn(ECHO_PIN, HIGH);
  Serial.print("Raw Duration: "); Serial.println(duration);

  // If no response, print error
  if (duration == 0) {
    Serial.println("Error: No Echo received!");
  } else {
    distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(500);


  return distance;
}


void loop() {
  
    float getdistance;
    getdistance = distance();
    
    if(getdistance < 5.00)
    {
        send();
    }
    else{
        mySerial.println("arri");
        delay(1000);
    
    }

    if (mySerial.available()) {
        String receivedData = mySerial.readStringUntil('\n'); // Read incoming message
        Serial.println("Received from Nano: " + receivedData);
    }
    delay(1000);

}
