#include <Arduino.h>
#include <coloursensor.h>


#define S0 21
#define S1 22
#define S2 18
#define S3 19
#define sensorOut 23



void setupcoloursensor() {
    
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
 
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}
String getcolour() {
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
    String colour = "YELLOW";
    Serial.println("Detected Color: YELLOW");
    delay(500);
    return colour;
    
    
    
  } else {
    String colour = "NOT YELLOW";
    Serial.println("Detected Color: NOT YELLOW");
    delay(500);
    return colour;
    
    
  }
  

  
}