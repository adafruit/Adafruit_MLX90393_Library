/***************************************************************************
 * Demo of sending magnetometer sensor data to a calibration software. To be
 * used with MotionCal and Jupyter Notebook examples found in this tutorial:
 * https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/
 * 
 * Note that this demo does not include accelerometer or gyroscope data. It
 * is just for sending raw magnetometer data to be used in the calibration
 * process.
 * 
 * An example of using calibrated magnetometer data can be found in
 * compass_calibrated.ino.
 * 
 * Author: Shawn Hymel
 * Date: February 20, 2022
 * 
 * Based on imucal by PJRC and adapted by Limor Fried for Adafruit Industries
 ***************************************************************************/

#include "Adafruit_MLX90393.h"

// Sensor object
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

void setup() {

  // Start serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Say hello
  Serial.println("MLX90393 Calibration");

  // Connect to sensor
  if (!mlx.begin_I2C()) {
    Serial.println("ERROR: Could not connect to magnetometer");
    while(1);
  }

  // Configure MLX90393
  mlx.setGain(MLX90393_GAIN_1X);
  mlx.setResolution(MLX90393_X, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Y, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Z, MLX90393_RES_16);
  mlx.setOversampling(MLX90393_OSR_3);
  mlx.setFilter(MLX90393_FILTER_5);
}

void loop() {

  static float x, y, z;

  // Sample data from magnetometer
  if (mlx.readData(&x, &y, &z)) {

    // Print raw data out for MotionCal (no accel or gyro data)
    Serial.print("Raw:");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(int(x*10)); Serial.print(",");
    Serial.print(int(y*10)); Serial.print(",");
    Serial.print(int(z*10)); Serial.println("");

    // Unified data (no accel or gyro data)
    Serial.print("Uni:");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(0); Serial.print(",");
    Serial.print(int(x)); Serial.print(",");
    Serial.print(int(y)); Serial.print(",");
    Serial.print(int(z)); Serial.println("");

  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }

  // Wait some
  delay(10);
}