#include <Arduino.h>

// #define S0 18
// #define S1 19
// #define S2 5
// #define S3 17
// #define sensorOut 16

// void setup() {
//   Serial.begin(115200);

//   pinMode(S0, OUTPUT);
//   pinMode(S1, OUTPUT);
//   pinMode(S2, OUTPUT);
//   pinMode(S3, OUTPUT);
//   pinMode(sensorOut, INPUT);

//   // Set frequency scaling to 20% for stable readings
//   digitalWrite(S0, HIGH);
//   digitalWrite(S1, LOW);
// }

// void loop() {
//   int red, green, blue;

//   // Select RED filter
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, LOW);
//   red = pulseIn(sensorOut, LOW);

//   // Select GREEN filter
//   digitalWrite(S2, HIGH);
//   digitalWrite(S3, HIGH);
//   green = pulseIn(sensorOut, LOW);

//   // Select BLUE filter
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, HIGH);
//   blue = pulseIn(sensorOut, LOW);

//   // Print RGB values
//   Serial.print("R: "); Serial.print(red);
//   Serial.print(" G: "); Serial.print(green);
//   Serial.print(" B: "); Serial.println(blue);

//   delay(500);
// }
// #define S0 21
// #define S1 22
// #define S2 18
// #define S3 19
// #define sensorOut 23

// void setup() {
//   Serial.begin(115200);
  
//   pinMode(S0, OUTPUT);
//   pinMode(S1, OUTPUT);
//   pinMode(S2, OUTPUT);
//   pinMode(S3, OUTPUT);
//   pinMode(sensorOut, INPUT);
  
//   // Set frequency scaling to 20% for stable readings
//   digitalWrite(S0, HIGH);
//   digitalWrite(S1, LOW);
// }

// void loop() {
//   int red, green, blue;
  
//   // Read RED
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, LOW);
//   delay(100);
//   red = pulseIn(sensorOut, LOW);

//   // Read GREEN
//   digitalWrite(S2, HIGH);
//   digitalWrite(S3, HIGH);
//   delay(100);
//   green = pulseIn(sensorOut, LOW); 

//   // Read BLUE
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, HIGH);
//   delay(100);
//   blue = pulseIn(sensorOut, LOW);

//   // Print values
//   Serial.print("Red: "); Serial.print(red);
//   Serial.print(" | Green: "); Serial.print(green);
//   Serial.print(" | Blue: "); Serial.println(blue);

//   delay(500);
// }

#define S0 21
#define S1 22
#define S2 18
#define S3 19
#define sensorOut 23

void setup() {
  Serial.begin(115200);  // Ensure Serial Monitor is set to 115200 baud
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  int red, green, blue;

  // Read RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(100);
  red = pulseIn(sensorOut, LOW);
  
  // Read GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(100);
  green = pulseIn(sensorOut, LOW); 

  // Read BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(100);
  blue = pulseIn(sensorOut, LOW);

  // Print RGB values
  Serial.print("Red: "); Serial.print(red);
  Serial.print(" | Green: "); Serial.print(green);
  Serial.print(" | Blue: "); Serial.println(blue);

  // Detect Yellow: High Red & Green, Low Blue
  if (red <70 && green >80 &&  green < 160 && blue >85 && blue < 130) {
    Serial.println("Detected Color: YELLOW");
  } else {
    Serial.println("Detected Color: NOT YELLOW");
  }

  delay(500);
}
