#include <Wire.h>

#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

void setup(void)
{
  Serial.begin(9600);

  /* Wait for serial on USB platforms. */
  while(!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90393 Demo");

  if (sensor.begin())
  {
    Serial.println("Found a MLX90393 sensor");
  }
  else
  {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
}

void loop(void)
{
    float x, y, z;

    if(sensor.readData(&x, &y, &z)) {
        Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
        Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
        Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
    } else {
        Serial.println("Unable to read XYZ data from the sensor.");
    }

    delay(500);
}
