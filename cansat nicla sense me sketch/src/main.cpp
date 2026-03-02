#include <Arduino.h>
#include <Arduino_BHY2.h>

void setup() {
  BHY2.begin();
  BHY2.setLDOTimeout(3600000);

}

void loop() {
  BHY2.update();
  delay(10);
}