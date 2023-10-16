/***************************************************************************
 * Demo of computing the heading (of the +X axis) from magnetometer data
 * obtained from an MLX90393. Hard-iron and soft-iron calibration offsets
 * and scaling are applied to account for such distortions. Additionally,
 * the magnetic declination can be applied to convert magnetic heading to
 * geographic heading.
 * 
 * Leave everything as default to apply no calibration data or declination.
 * 
 * Run magcal_nosave.ino example to send raw data to MotionCal, following
 * this tutorial:
 * https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/
 * 
 * Copy the hard-iron and soft-iron calibration data to hard_iron[] and
 * soft_iron[][] arrays. Find your location from 
 * https://www.magnetic-declination.com/. Copy the "Magnetic Declination"
 * angle value to mag_decl if you want to convert magnetic heading to
 * geographic heading.
 * 
 * Author: Shawn Hymel
 * Date: February 20, 2022
 * 
 * Based on the Adafruit_LIS2MDL compass demo by Limor Fried
 ***************************************************************************/

#include "Adafruit_MLX90393.h"

// Hard-iron calibration settings
// Set to {0, 0, 0} for no hard-iron offset
const float hard_iron[3] = { 
     0.0,   0.0,    0.0
};

// Soft-iron calibration settings
// Set to identity matrix for no soft-iron scaling
const float soft_iron[3][3] = {
  {  1.0,    0.0,    0.0    },
  {  0.0,    1.0,    0.0  },
  {  0.0,    0.0,    1.0  }
};

// Magnetic declination from magnetic-declination.com
// East is positive (+), west is negative (-)
// mag_decl = (+/-)(deg + min/60 + sec/3600)
// Set to 0 to get magnetic heading instead of geographic heading
const float mag_decl = 0.0;

// Sensor object
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

void setup() {

  // Start serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Say hello
  Serial.println("Starting Adafruit MLX90393 Calibrated Compass");

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

  // Raw magnetometer data stored as {x, y, z}
  static float mag_data[3] = {0.0, 0.0, 0.0};

  static float hi_cal[3];
  static float heading;

  // Sample and compensate with hard/soft-iron calibration data
  if (mlx.readData(&mag_data[0], &mag_data[1], &mag_data[2])) {

    // Apply hard-iron offsets
    for (uint8_t i = 0; i < 3; i++) {
      hi_cal[i] = mag_data[i] - hard_iron[i];
    }

    // Apply soft-iron scaling
    for (uint8_t i = 0; i < 3; i++) {
      mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + 
                    (soft_iron[i][1] * hi_cal[1]) +
                    (soft_iron[i][2] * hi_cal[2]);
    }

    // Calculate angle for heading, assuming board is parallel to
    // the ground and +X points toward heading.
    // WARNING: X and Y silkscreen marketings are backward on v1 of board
    heading = (atan2(mag_data[1], mag_data[0]) * 180) / M_PI;

    // Apply magnetic declination to convert magnetic heading
    // to geographic heading
    heading += mag_decl;

    // Normalize to 0..360
    if (heading < 0) {
      heading += 360;
    }

    // Print raw readings and heading
    Serial.print("X: ");
    Serial.print(mag_data[0], 2);
    Serial.print("\tY: ");
    Serial.print(mag_data[1], 2);
    Serial.print("\tZ: ");
    Serial.print(mag_data[2], 2);
    Serial.print("\tHeading: ");
    Serial.println(heading, 2);

  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }

  // Wait some
  delay(10);
}