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
Adafruit_MLX90393::Adafruit_MLX90393(TwoWire* wireBus)
{
    /* Set the I2C bus instance */
    _wire        = wireBus;
    _transport   = MLX90393_TRANSPORT_I2C;
    _initialized = false;
    _gain        = MLX90393_GAIN_1X;
    _i2caddr     = 0;
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
    bool ok;

    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->begin();
            _i2caddr = i2caddr;
            break;
        case MLX90393_TRANSPORT_SPI:
            /* Currently not handled due to HW layout. */
            break;
    }

    /* Set gain and sensor config. */
    ok = setGain(_gain);

    /* Set the initialised flag based on the I2C response. */
    _initialized = ok;

    return ok;
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
    bool ok;

    _gain = gain;

    /* Set CONF1, including gain. */
    uint8_t tx[4] = { MLX90393_REG_WR,
                      0x00,
                      (uint8_t)(((_gain & 0x7) << MLX90393_GAIN_SHIFT) | MLX90393_HALL_CONF),
                      (MLX90393_CONF1 & 0x3F) << 2};

    /* Perform the transaction. */
    ok = transceive(tx, sizeof tx, NULL, 0);

    /* Check status byte for errors. */
    return ok;
}

/**
 * Sets the TRIG_INT pin to the specified function.
 *
 * @param state  'true/1' sets the pin to INT, 'false/0' to TRIG.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool
Adafruit_MLX90393::setTrigInt(bool state)
{
    bool ok;
    uint8_t trig_int = 0;

    if (state) {
        /* Set the INT (bit 8 = 1). */
        trig_int = 0x80;
    }

    /* Set CONF2 bit 15 to 'state' */
    uint8_t tx[4] = { MLX90393_REG_WR,
                      0x00,                 /* Lower 8 bits */
                      trig_int,             /* Upper 8 bits */
                      (MLX90393_CONF2 & 0x3f) << 2};
    if (state) {
        /* Set TRIG_INT to INT, from default value of TRIG. */
        tx[1] = 0x80;
    };

    /* Perform the transaction. */
    ok = transceive(tx, sizeof tx, NULL, 0);

    /* Check status byte for errors. */
    return ok;
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
    bool ok;
    uint8_t tx_mode[1] = { MLX90393_REG_SM | MLX90393_AXIS_ALL };
    uint8_t tx[1] = { MLX90393_REG_RM | MLX90393_AXIS_ALL };
    uint8_t rx[6] = { 0 };
    int16_t xi, yi, zi;

    /* Set the device to single measurement mode */
    ok = transceive(tx_mode, sizeof tx_mode, NULL, 0);
    if (!ok) {
        return false;
    }

    /* Read a single data sample. */
    ok = transceive(tx, sizeof tx, rx, sizeof rx);
    if (!ok) {
        return false;
    }

    /* Convert data to uT and float. */
    xi = (rx[0] << 8) | rx[1];
    yi = (rx[2] << 8) | rx[3];
    zi = (rx[4] << 8) | rx[5];

    *x = (float)xi * mlx90393_lsb_lookup[_gain][0][0];
    *y = (float)yi * mlx90393_lsb_lookup[_gain][0][0];
    *z = (float)zi * mlx90393_lsb_lookup[_gain][0][1];

    return ok;
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
bool
Adafruit_MLX90393::transceive(uint8_t *txbuf, uint8_t txlen,
    uint8_t *rxbuf, uint8_t rxlen)
{
    uint8_t status = 0;
    uint8_t i;

    /* Write stage */
    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->beginTransmission(_i2caddr);
            for (i = 0; i < txlen; i++) {
                _wire->write(txbuf[i]);
            }
            _wire->endTransmission();
            /* Wait a bit befoore requesting a response. */
            delay(10);
            break;
        case MLX90393_TRANSPORT_SPI:
            /* Currently not handled due to HW layout. */
            break;
    }

    /* Read stage. */
    switch(_transport) {
        case MLX90393_TRANSPORT_I2C:
            _wire->requestFrom(_i2caddr, (uint8_t)((rxlen + 1) & 0xFF));
            /* Always request the status byte. */
            status = _wire->read();
            /* Read any other bytes that have been requested. */
            if (rxbuf != NULL) {
                for (i = 0; i < rxlen; i++) {
                    rxbuf[i] = _wire->read();
                }
            }
            break;
        case MLX90393_TRANSPORT_SPI:
            /* Currently not handled due to HW layout. */
            break;
    }

    /* Mask out bytes available in the status response. */
    return ((status >> 2) == 0 ? true : false);
}
