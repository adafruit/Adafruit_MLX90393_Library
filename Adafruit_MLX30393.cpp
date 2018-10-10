/******************************************************************************
  This is a library for the MLX90393 magnetometer.

  Designed specifically to work with the MLX90393 breakout from Adafruit:

  ----> https://www.adafruit.com/products/XXXX

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code, please
  support Adafruit and open-source hardware by purchasing products from
  Adafruit!

  Written by Kevin Townsend/ktown for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 *****************************************************************************/
#include "Adafruit_MLX90393.h"

/**
 * Instantiates a new Adafruit_MLX90393 class instance.
 *
 * @param sensorID  An optional ID # so you can track this sensor, it will
 *                  tag sensorEvents you create.
 *
 */
Adafruit_MLX90393::Adafruit_MLX90393(int32_t sensorID)
{
    /* Set the I2C bus instance */
    _wire        = &Wire;
    _transport   = MLX90393_TRANSPORT_I2C;
    _initialized = false;
    _gain        = MLX90393_GAIN_1X;
    _sensorID    = sensorID;
}

/**
 * Instantiates a new Adafruit_MLX90393 class instance.
 *
 * @param sensorID  An optional ID # so you can track this sensor, it will
 *                  tag sensorEvents you create.
 * @param wireBus   TwoWire instance to use for I2C communication.
 */
Adafruit_MLX90393::Adafruit_MLX90393(int32_t sensorID, TwoWire* wireBus)
{
    /* Set the I2C bus instance */
    _wire        = wireBus;
    _transport   = MLX90393_TRANSPORT_I2C;
    _initialized = false;
    _gain        = MLX90393_GAIN_1X;
    _sensorID    = sensorID;
}

/**
 * Initialises the I2C bus, and assigns the I2C address to us.
 *
 * @param i2caddr   The I2C address to use for the sensor.
 *
 * @return True if initialisation was successful, otherwise False.
 */
bool
Adafruit_MLX90393::begin(uint8_t i2caddr)
{
    bool rc;

    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->begin();
            _i2caddr = i2caddr;
            break;
        case MLX90393_TRANSPORT_SPI:
            /* TODO! */
            break;
    }

    /* Set gain and config sensor. */
    rc = setGain(_gain);

    if (rc) {
        _initialized = false;
        return false;
    } else {
        _initialized = true;
        return true;
    }
}

/**
 * Sets the sensor gain to the specified level.
 *
 * @param gain  The gain level to set.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool
Adafruit_MLX90393::setGain(enum mlx90393_gain gain)
{
    uint8_t status;

    _gain = gain;

    /* Set CONF1..4, including gain. */
    uint8_t tx[4] = { 0x60,
                      0x00,
                      _gain << MLX90393_GAIN_SHIFT | MLX90393_HALL_CONF,
                      0x00 };

    /* Perform the transaction. */
    status = transceive(tx, sizeof tx, NULL, 0);

    /* Check status byte for errors. */
    if ((status & MLX90393_STATUS_MASK) != MLX90393_STATUS_OK) {
        return false;
    } else {
        return true;
    }
}

/**
 * Gets the current sensor gain.
 *
 * @return An enum containing the current gain level.
 */
enum mlx90393_gain
Adafruit_MLX90393::getGain(void)
{
    return _gain;
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
bool
Adafruit_MLX90393::readData(float *x, float *y, float *z)
{
    uint8_t status;
    uint8_t tx[1] = { MLX90393_REG_SM | MLX90393_AXIS_ALL };
    uint8_t rx[6] = { 0 };
    uint16_t xi, yi, zi;

    /* Get a single data sample. */
    status = transceive(tx, sizeof tx, rx, sizeof rx);
    if ((status & MLX90393_STATUS_MASK) != MLX90393_STATUS_OK) {
        return false;
    }

    /* Convert data to uT and float. */
    xi = (rx[0] << 8) | rx[1];
    yi = (rx[2] << 8) | rx[3];
    zi = (rx[4] << 8) | rx[5];

    *x = (float)xi * mlx90393_lsb_lookup[_gain][0][0];
    *y = (float)yi * mlx90393_lsb_lookup[_gain][0][0];
    *z = (float)zi * mlx90393_lsb_lookup[_gain][0][1];

    return true;
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
uint8_t
Adafruit_MLX90393::transceive(uint8_t *txbuf, uint8_t txlen,
    uint8_t *rxbuf, uint8_t rxlen)
{
    uint8_t status;
    uint8_t i;

    /* Write stage */
    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->beginTransmission(_i2caddr);
            for (i = 0; i < txlen; i++)
            {
                _wire->write(txbuf[i]);
            }
            _wire->endTransmission();
            /* Wait a bit befoore requesting a response. */
            delay(10);
            break;
        case MLX90393_TRANSPORT_SPI:
            #if 0
            digitalWrite(_cs, LOW);
            SPI.transfer((reg << 1) | 0x00);
            for (uint8_t i = 0; i < len; i++){
              SPI.transfer(buffer[i]);
            }
            digitalWrite(_cs, HIGH);
            #endif
            break;
    }

    /* Read stage. */
    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->requestFrom(_i2caddr, rxlen + 1);
            /* Always request the status byte. */
            status = _wire->read();
            /* Read any other bytes that have been requested. */
            for (i = 0; i < rxlen; i++)
            {
                rxbuf[i] = _wire->read();
            }
            break;
        case MLX90393_TRANSPORT_SPI:
            break;
    }

    return status;
}
