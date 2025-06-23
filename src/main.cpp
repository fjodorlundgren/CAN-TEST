#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Teensy 4.0 Serial Test ===");
  Serial.println("If you see this, Serial is working!");
}

void loop()
{
  Serial.print("Loop count: ");
  Serial.println(millis());
  delay(1000);
}
