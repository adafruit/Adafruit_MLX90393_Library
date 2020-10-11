/******************************************************************************
  This is a library for the MLX90393 magnetometer.

  Designed specifically to work with the MLX90393 breakout from Adafruit:

  ----> https://www.adafruit.com/products/4022

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code, please
  support Adafruit and open-source hardware by purchasing products from
  Adafruit!

  Written by Kevin Townsend/ktown for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 *****************************************************************************/
#include "Adafruit_MLX90393.h"

/**
 * Instantiates a new Adafruit_MLX90393 class instance using I2C.
 *
 * @param wireBus   TwoWire instance to use for I2C communication.
 */
Adafruit_MLX90393::Adafruit_MLX90393(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MLX90393::begin_I2C(uint8_t i2c_addr, TwoWire *wire) {
  if (i2c_dev) {
    delete i2c_dev;
  }

  if (!i2c_dev) {
    i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  }
  spi_dev = NULL;

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

bool Adafruit_MLX90393::_init(void) {

  if (!exitMode())
    return false;

  if (!reset())
    return false;

  /* Set gain and sensor config. */
  if (!setGain(MLX90393_GAIN_1X)) {
    return false;
  }
  return true;
}

/**
 * Perform a mode exit
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::exitMode(void) {
  uint8_t tx[1] = {MLX90393_REG_EX};

  /* Perform the transaction. */
  return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
}

/**
 * Perform a soft reset
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::reset(void) {
  uint8_t tx[1] = {MLX90393_REG_RT};

  /* Perform the transaction. */
  if (transceive(tx, sizeof(tx), NULL, 0, 5) != MLX90393_STATUS_RESET) {
    return false;
  }
  return true;
}

/**
 * Sets the sensor gain to the specified level.
 * @param gain  The gain level to set.
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::setGain(enum mlx90393_gain gain) {
  _gain = gain;

  /* Set CONF1, including gain. */
  uint8_t tx[4] = {
      MLX90393_REG_WR, 0x00,
      (uint8_t)(((_gain & 0x7) << MLX90393_GAIN_SHIFT) | MLX90393_HALL_CONF),
      (MLX90393_CONF1 & 0x3F) << 2};

  /* Perform the transaction. */
  return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
}

/**
 * Sets the TRIG_INT pin to the specified function.
 *
 * @param state  'true/1' sets the pin to INT, 'false/0' to TRIG.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::setTrigInt(bool state) {
  bool ok;
  uint8_t trig_int = 0;

  if (state) {
    /* Set the INT (bit 8 = 1). */
    trig_int = 0x80;
  }

  /* Set CONF2 bit 15 to 'state' */
  uint8_t tx[4] = {MLX90393_REG_WR, 0x00, /* Lower 8 bits */
                   trig_int,              /* Upper 8 bits */
                   (MLX90393_CONF2 & 0x3f) << 2};

  /* Perform the transaction. */
  return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
}

/**
 * Gets the current sensor gain.
 *
 * @return An enum containing the current gain level.
 */
enum mlx90393_gain Adafruit_MLX90393::getGain(void) { return _gain; }

/**
 * Begin a single measurement on all axes
 *
 * @return True on command success
 */
bool Adafruit_MLX90393::startSingleMeasurement(void) {
  uint8_t tx[1] = {MLX90393_REG_SM | MLX90393_AXIS_ALL};

  /* Set the device to single measurement mode */
  if (transceive(tx, sizeof(tx)) != MLX90393_STATUS_OK) {
    return false;
  }
  return true;
}

/**
 * Reads data from data register & returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::readMeasurement(float *x, float *y, float *z) {
  uint8_t tx[1] = {MLX90393_REG_RM | MLX90393_AXIS_ALL};
  uint8_t rx[6] = {0};

  /* Read a single data sample. */
  if (transceive(tx, sizeof(tx), rx, sizeof(rx), 0) != MLX90393_STATUS_OK) {
    return false;
  }

  int16_t xi, yi, zi;

  /* Convert data to uT and float. */
  xi = (rx[0] << 8) | rx[1];
  yi = (rx[2] << 8) | rx[3];
  zi = (rx[4] << 8) | rx[5];

  *x = (float)xi * mlx90393_lsb_lookup[0][_gain][0][0];
  *y = (float)yi * mlx90393_lsb_lookup[0][_gain][0][0];
  *z = (float)zi * mlx90393_lsb_lookup[0][_gain][0][1];

  return true;
}

/**
 * Performs a single X/Y/Z conversion and returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::readData(float *x, float *y, float *z) {
  if (!startSingleMeasurement())
    return false;

  return readMeasurement(x, y, z);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  event Pointer to an Adafruit Unified sensor_event_t object that
   we'll fill in
    @returns True on successful read
*/
/**************************************************************************/
bool Adafruit_MLX90393::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = 1;
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();

  return readData(&event->magnetic.x, &event->magnetic.y, &event->magnetic.z);
}

/**
 * Performs a full read/write transaction with the sensor.
 *
 * @param txbuf     Pointer the the buffer containing the data to write.
 * @param txlen     The number of bytes to write.
 * @param rxbuf     Pointer to an appropriately large buffer where data read
 *                  back will be written.
 * @param rxlen     The number of bytes to read back (not including the
 *                  mandatory status byte that is always returned).
 *
 * @return The status byte from the IC.
 */
bool Adafruit_MLX90393::transceive(uint8_t *txbuf, uint8_t txlen,
                                   uint8_t *rxbuf, uint8_t rxlen,
                                   uint8_t interdelay) {
  uint8_t status = 0;
  uint8_t i;
  uint8_t rxbuf2[rxlen + 1];

  /* Write stage */
  if (i2c_dev) {
    if (!i2c_dev->write(txbuf, txlen)) {
      return MLX90393_STATUS_ERROR;
    }
  }

  delay(interdelay);

  /* Read stage. */
  if (i2c_dev) {
    /* Read status byte plus any others */
    if (!i2c_dev->read(rxbuf2, rxlen + 1)) {
      return MLX90393_STATUS_ERROR;
    }
    status = rxbuf2[0];
    for (i = 0; i < rxlen; i++)
      rxbuf[i] = rxbuf2[i + 1];
  }

  /* Mask out bytes available in the status response. */
  return (status >> 2);
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t device data, Adafruit Unified Sensor format
    @param  sensor Pointer to an Adafruit Unified sensor_t object that we'll
   fill in
*/
/**************************************************************************/
void Adafruit_MLX90393::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MLX90393", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -50000; // -50 gauss in uTesla
  sensor->max_value = 50000;  // +50 gauss in uTesla
  sensor->resolution = 0.15;  // 100/16-bit uTesla per LSB
}
