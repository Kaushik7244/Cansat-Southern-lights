#include <Arduino.h>
#include <Arduino_BHY2.h>

void setup() {
  Serial.begin(115200);
  while (!BHY2.begin());
  BHY2.setLDOTimeout(3600000);

}

void loop() {

  BHY2.update();
  delay(1);
}