/*

* Basic BPM Measurement with Pulse Sensor

* Quick Test Sketch for Arduino Uno/Nano

* Adapted code - Thanks to Floris Wouterlood

* T.K.Hareendran/2019

*/
#include <Arduino.h>



int sensorPin = 36;                                // A0 is the input pin for the heart rate sensor
float sensorValue = 0;                             // Variable to store the value coming from the sensor
int count = 9;
unsigned long starttime = 0;
int heartrate = 0;
bool counted = false;
int LED13=2;
int Threshold = 2010; 




void setup (void) {
  pinMode(LED13, OUTPUT);                              // D13 LED as Status Indicator

  Serial.begin (115200);                               // Start Serial Communication @ 9600

}




void loop (){

starttime = millis();

while (millis()<starttime+5000){                  // Reading pulse sensor for 10 seconds

sensorValue = analogRead(sensorPin);
Serial.println(sensorValue);
if (sensorValue > Threshold && counted == false){  // Threshold value is 550 (~ 2.7V)

count++;
// Serial.print ("count = ");
// Serial.println (count);
digitalWrite (LED13,HIGH);
// delay (50);
// digitalWrite (LED13, LOW);
counted = true;

}

else if (sensorValue < Threshold){

counted = false;
digitalWrite (LED13, LOW);
}

}




heartrate = count*6;                               // Multiply the count by 6 to get beats per minute
// Serial.println ();
// Serial.print ("BPM = ");
// Serial.println (heartrate);                        // Display BPM in the Serial Monitor
// Serial.println ();
count = 0;
}