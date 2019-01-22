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
#ifndef ADAFRUIT_MLX90393_H
#define ADAFRUIT_MLX90393_H

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#define MLX90393_DEFAULT_ADDR   (0x0C)  /* Can also be 0x18, depending on IC */

#define MLX90393_AXIS_ALL       (0x0E)  /**< X+Y+Z axis bits for commands. */
#define MLX90393_CONF1          (0x00)  /**< Gain */
#define MLX90393_CONF2          (0x01)  /**< Burst, comm mode */
#define MLX90393_CONF3          (0x02)  /**< Oversampling, filter, res. */
#define MLX90393_CONF4          (0x03)  /**< Sensitivty drift. */
#define MLX90393_GAIN_SHIFT     (4)     /**< Left-shift for gain bits. */
#define MLX90393_RES_2_15       (0x00)  /**< Resolution (2^15). */
#define MLX90393_HALL_CONF      (0x0C)  /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK      (0x00)  /**< OK value for status response. */
#define MLX90393_STATUS_MASK    (0xFC)  /**< Mask for status OK checks. */

/** Register map. */
enum {
    MLX90393_REG_SB           = (0x10), /**< Start burst mode. */
    MLX90393_REG_SW           = (0x20), /**< Start wakeup on change mode. */
    MLX90393_REG_SM           = (0x30), /**> Start single-meas mode. */
    MLX90393_REG_RM           = (0x40), /**> Read measurement. */
    MLX90393_REG_RR           = (0x50), /**< Read register. */
    MLX90393_REG_WR           = (0x60), /**< Write register. */
    MLX90393_REG_EX           = (0x80), /**> Exit moode. */
    MLX90393_REG_HR           = (0xD0), /**< Memory recall. */
    MLX90393_REG_HS           = (0x70), /**< Memory store. */
    MLX90393_REG_RT           = (0xF0), /**< Reset. */
    MLX90393_REG_NOP          = (0x00), /**< NOP. */
};

/** Gain settings for CONF1 register. */
enum  mlx90393_gain {
    MLX90393_GAIN_5X          = (0x00),
    MLX90393_GAIN_4X,
    MLX90393_GAIN_3X,
    MLX90393_GAIN_2_5X,
    MLX90393_GAIN_2X,
    MLX90393_GAIN_1_67X,
    MLX90393_GAIN_1_33X,
    MLX90393_GAIN_1X
};

/** Transport options for this sensor. */
enum mlx90393_transport {
    MLX90393_TRANSPORT_I2C      = (0x00),
    MLX90393_TRANSPORT_SPI
};

/** Lookup table to convert raw values too uT based on gain and res. */
const float mlx90393_lsb_lookup[8][4][2] = {
    /* 5x gain */
    {{0.805, 1.468}, {1.610, 2.936}, {3.220, 5.872}, {6.440, 11.744}},
    /* 4x gain */
    {{0.644, 1.174}, {1.288, 2.349}, {2.576, 4.698}, {5.152, 9.395}},
    /* 3x gain */
    {{0.483, 0.881}, {0.966, 1.762}, {1.932, 3.523}, {3.864, 7.046}},
    /* 2.5x gain */
    {{0.403, 0.734}, {0.805, 1.468}, {1.610, 2.936}, {3.220, 5.872}},
    /* 2x gain */
    {{0.322, 0.587}, {0.644, 1.174}, {1.288, 2.349}, {2.576, 4.698}},
    /* 1.667x gain */
    {{0.268, 0.489}, {0.537, 0.979}, {1.073, 1.957}, {2.147, 3.915}},
    /* 1.333x gain */
    {{0.215, 0.391}, {0.429, 0.783}, {0.859, 1.566}, {1.717, 3.132}},
    /* 1x gain */
    {{0.161, 0.294}, {0.322, 0.587}, {0.644, 1.174}, {1.288, 2.349}}
};

/**
 * Driver for the Adafruit SHT31-D Temperature and Humidity breakout board.
 */
class Adafruit_MLX90393 {
    public:
        Adafruit_MLX90393(TwoWire* wireBus = &Wire);

        bool begin(uint8_t i2caddr = MLX90393_DEFAULT_ADDR);
        bool setGain(enum mlx90393_gain gain);
        bool setTrigInt(bool state);
        enum mlx90393_gain getGain(void);
        bool readData(float *x, float *y, float *z);

    private:
        enum mlx90393_transport _transport;
        enum mlx90393_gain _gain;
        TwoWire* _wire;
        bool _initialized;
        uint8_t _i2caddr;

        bool transceive(uint8_t *txbuf, uint8_t txlen,
                        uint8_t *rxbuf, uint8_t rxlen);
};

#endif /* ADAFRUIT_MLX90393_H */
