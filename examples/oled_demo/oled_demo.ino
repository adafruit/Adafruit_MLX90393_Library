#include <Adafruit_SSD1306.h>
#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("MLX demo");

  if (sensor.begin_I2C()) {
    Serial.println("Found a MLX90393 sensor");
  } else {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(500); // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
}


void loop() {
  float x, y, z;

  display.clearDisplay();
  if(sensor.readData(&x, &y, &z)) {
    Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
    Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
    Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");

     display.setCursor(0,0);
     display.println("- Adafruit MLX90393 -");
     display.print("X:"); display.print(x, 1); display.println(" uT");
     display.print("Y:"); display.print(y, 1); display.println(" uT");
     display.print("Z:"); display.print(z, 1); display.println(" uT");
  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }
  display.display();
  delay(50);
}

