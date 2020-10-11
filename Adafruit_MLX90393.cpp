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


/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_MLX90393::begin_SPI(uint8_t cs_pin, SPIClass *theSPI) {
  i2c_dev = NULL;
  if (!spi_dev) {
    _cspin = cs_pin;
    spi_dev = new Adafruit_SPIDevice(cs_pin,
                                     1000000,               // frequency
                                     SPI_BITORDER_MSBFIRST, // bit order
                                     SPI_MODE3,             // data mode
                                     theSPI);
  }
  if (!spi_dev->begin()) {
    return false;
  }
  return _init();
}

/*!
 *    @brief  Sets up the hardware and initializes software SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  sck_pin The arduino pin # connected to SPI clock
 *    @param  miso_pin The arduino pin # connected to SPI MISO
 *    @param  mosi_pin The arduino pin # connected to SPI MOSI
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MLX90393::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                                 int8_t mosi_pin) {
  i2c_dev = NULL;
  if (!spi_dev) {
    _cspin = cs_pin;

    spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                     1000000,               // frequency
                                     SPI_BITORDER_MSBFIRST, // bit order
                                     SPI_MODE3);            // data mode
  }
  if (!spi_dev->begin()) {
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

  // set INT pin to output interrupt
  if (! setTrigInt(false)) {
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
bool Adafruit_MLX90393::setGain(mlx90393_gain_t gain) {
  _gain = gain;

  uint16_t data;
  readRegister(MLX90393_CONF1, &data);

  // mask off gain bits
  data &= ~0x0070;
  // set gain bits
  data |= gain << MLX90393_GAIN_SHIFT;

  return writeRegister(MLX90393_CONF1, data);
}

/**
 * Gets the current sensor gain.
 *
 * @return An enum containing the current gain level.
 */
mlx90393_gain_t Adafruit_MLX90393::getGain(void) {
  uint16_t data;
  readRegister(MLX90393_CONF1, &data);

  // mask off gain bits
  data &= 0x0070;

  return (mlx90393_gain_t)(data >> 4);
}

/**
 * Sets the TRIG_INT pin to the specified function.
 *
 * @param state  'true/1' sets the pin to INT, 'false/0' to TRIG.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90393::setTrigInt(bool state) {
  uint16_t data;
  readRegister(MLX90393_CONF2, &data);

  // mask off trigint bit
  data &= ~0x8000;

  // set trigint bit if desired
  if (state) {
    /* Set the INT, highest bit */
    data |= 0x8000;
  }

  return writeRegister(MLX90393_CONF2, data);
}

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

bool Adafruit_MLX90393::writeRegister(uint8_t reg, uint16_t data) {
  uint8_t tx[4] = {MLX90393_REG_WR,
                   data >> 8,   // high byte
                   data & 0xFF, // low byte
                   reg << 2};   // the register itself, shift up by 2 bits!

  /* Perform the transaction. */
  return (transceive(tx, sizeof(tx), NULL, 0, 0) == MLX90393_STATUS_OK);
}

bool Adafruit_MLX90393::readRegister(uint8_t reg, uint16_t *data) {
  uint8_t tx[2] = {MLX90393_REG_RR,
                   reg << 2}; // the register itself, shift up by 2 bits!

  uint8_t rx[2];

  /* Perform the transaction. */
  if (transceive(tx, sizeof(tx), rx, sizeof(rx), 0) != MLX90393_STATUS_OK) {
    return false;
  }

  *data = ((uint16_t)rx[0] << 8) | rx[1];

  return true;
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
  uint8_t rxbuf2[rxlen + 2];

  if (i2c_dev) {
    /* Write stage */
    if (!i2c_dev->write(txbuf, txlen)) {
      return MLX90393_STATUS_ERROR;
    }
    delay(interdelay);

    /* Read status byte plus any others */
    if (!i2c_dev->read(rxbuf2, rxlen + 1)) {
      return MLX90393_STATUS_ERROR;
    }
    status = rxbuf2[0];
    for (i = 0; i < rxlen; i++) {
      rxbuf[i] = rxbuf2[i + 1];
    }
  }


  if (spi_dev) {
    spi_dev->write_then_read(txbuf, txlen, rxbuf2, rxlen+1, 0x00);
    status = rxbuf2[0];
    for (i = 0; i < rxlen; i++) {
      rxbuf[i] = rxbuf2[i + 1];
    }

    /*
    spi_dev->beginTransaction();
    digitalWrite(_cspin, LOW);


    Serial.print("SPI Write: ");
    for (size_t i = 0; i < txlen; i++) {
      Serial.print(F("0x"));
      Serial.print(txbuf[i], HEX);
      Serial.print(F(", "));

      spi_dev->transfer(txbuf[i]);
    }
    Serial.println();

    delay(interdelay);

    status = spi_dev->transfer(0x0);
    Serial.print("SPI status: 0x");
    Serial.println(status, HEX);

    // do the reading
    Serial.print("SPI Read: ");
    for (size_t i = 0; i < rxlen; i++) {
      rxbuf[i] = spi_dev->transfer(0x0);
      Serial.print(F("0x"));
      Serial.print(rxbuf[i], HEX);
      Serial.print(F(", "));
    }
    Serial.println();
    digitalWrite(_cspin, HIGH);
    spi_dev->endTransaction();
    */

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
