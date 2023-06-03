#include <Arduino.h>

void setup() {
    pinMode(4, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(5, INPUT_PULLUP);         // pin that will blink to your heartbeat!
    Serial.begin(115200);         // Set's up Serial Communication at certain speed.
    // attachInterrupt(5, isr1, FALLING);
    // attachInterrupt(5, isr2, RISING);
}

// The Main Loop Function
void loop() {
    (digitalRead(5)==0)? digitalWrite(4, HIGH): digitalWrite(4, LOW);
}