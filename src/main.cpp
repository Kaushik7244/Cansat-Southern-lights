#include <Arduino.h>


void setup() {
  pinMode(LEDR,1);
  pinMode(LEDG,1);
  pinMode(LEDB,1);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LEDR,1);
  digitalWrite(LEDG,1);
  digitalWrite(LEDB,1);
  delay(100);
}

