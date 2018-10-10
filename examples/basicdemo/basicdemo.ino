#include <Wire.h>

#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 mlx = Adafruit_MLX90393(393);

void setup(void)
{
  Serial.begin(9600);

  Serial.println(F("Starting Adafruit MLX90393 Tester!"));

  if (mlx.begin())
  {
    Serial.println(F("Found a MLX90393 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
}

void loop(void)
{
  delay(500);
}
