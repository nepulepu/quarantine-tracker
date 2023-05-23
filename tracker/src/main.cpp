#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
#ifndef LED_builtin
  #define LED_builtin 21
#endif

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.print(result);
  pinMode(LED_builtin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.print("up");
  digitalWrite(LED_builtin,HIGH);
  delay(1000);
  Serial.print("down");
  digitalWrite(LED_builtin,LOW);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}