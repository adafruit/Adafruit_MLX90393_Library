#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

void setup(void)
{
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
      delay(10);
  }

  Serial.println("Starting Adafruit MLX90393 Demo");

  if (sensor.begin_I2C()) {
    Serial.println("Found a MLX90393 sensor");
  } else {
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
}

void loop(void) {
  float x, y, z;

  // get X Y and Z data at once 
  if (sensor.readData(&x, &y, &z)) {
      Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
      Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
      Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }

  delay(500);

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  sensor.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("X: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
  Serial.println(" uTesla ");

  delay(500);
}