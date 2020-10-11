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
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

#define MLX90393_DEFAULT_ADDR (0x0C) /* Can also be 0x18, depending on IC */

#define MLX90393_AXIS_ALL (0x0E)     /**< X+Y+Z axis bits for commands. */
#define MLX90393_CONF1 (0x00)        /**< Gain */
#define MLX90393_CONF2 (0x01)        /**< Burst, comm mode */
#define MLX90393_CONF3 (0x02)        /**< Oversampling, filter, res. */
#define MLX90393_CONF4 (0x03)        /**< Sensitivty drift. */
#define MLX90393_GAIN_SHIFT (4)      /**< Left-shift for gain bits. */
#define MLX90393_RES_2_15 (0x00)     /**< Resolution (2^15). */
#define MLX90393_HALL_CONF (0x0C)    /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK (0x00)    /**< OK value for status response. */
#define MLX90393_STATUS_SMMODE (0x08)    /**< SM Mode status response. */
#define MLX90393_STATUS_RESET (0x01) /**< Reset value for status response. */
#define MLX90393_STATUS_ERROR (0xFF) /**< OK value for status response. */
#define MLX90393_STATUS_MASK (0xFC)  /**< Mask for status OK checks. */

/** Register map. */
enum {
  MLX90393_REG_SB = (0x10),  /**< Start burst mode. */
  MLX90393_REG_SW = (0x20),  /**< Start wakeup on change mode. */
  MLX90393_REG_SM = (0x30),  /**> Start single-meas mode. */
  MLX90393_REG_RM = (0x40),  /**> Read measurement. */
  MLX90393_REG_RR = (0x50),  /**< Read register. */
  MLX90393_REG_WR = (0x60),  /**< Write register. */
  MLX90393_REG_EX = (0x80),  /**> Exit moode. */
  MLX90393_REG_HR = (0xD0),  /**< Memory recall. */
  MLX90393_REG_HS = (0x70),  /**< Memory store. */
  MLX90393_REG_RT = (0xF0),  /**< Reset. */
  MLX90393_REG_NOP = (0x00), /**< NOP. */
};

/** Gain settings for CONF1 register. */
typedef enum mlx90393_gain {
  MLX90393_GAIN_5X = (0x00),
  MLX90393_GAIN_4X,
  MLX90393_GAIN_3X,
  MLX90393_GAIN_2_5X,
  MLX90393_GAIN_2X,
  MLX90393_GAIN_1_67X,
  MLX90393_GAIN_1_33X,
  MLX90393_GAIN_1X
} mlx90393_gain_t;

/** Lookup table to convert raw values to uT based on [HALLCONF][GAIN_SEL][RES].
 */
const float mlx90393_lsb_lookup[2][8][4][2] = {

    /* HALLCONF = 0xC (default) */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    /* HALLCONF = 0x0 */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }};

/**
 * Driver for the Adafruit SHT31-D Temperature and Humidity breakout board.
 */
class Adafruit_MLX90393 : public Adafruit_Sensor {
public:
  Adafruit_MLX90393();
  bool begin_I2C(uint8_t i2c_addr = MLX90393_DEFAULT_ADDR,
                 TwoWire *wire = &Wire);
  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin);

  bool reset(void);
  bool exitMode(void);

  bool readMeasurement(float *x, float *y, float *z);
  bool startSingleMeasurement(void);

  bool setGain(enum mlx90393_gain gain);
  enum mlx90393_gain getGain(void);

  bool setTrigInt(bool state);
  bool readData(float *x, float *y, float *z);

  bool getEvent(sensors_event_t *event);
  void getSensor(sensor_t *sensor);

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_SPIDevice *spi_dev = NULL;

  bool _init(void);
  enum mlx90393_gain _gain;
  bool readRegister(uint8_t reg, uint16_t *data);
  bool writeRegister(uint8_t reg, uint16_t data);

  uint8_t transceive(uint8_t *txbuf, uint8_t txlen, uint8_t *rxbuf = NULL,
                  uint8_t rxlen = 0, uint8_t interdelay = 10);

  int32_t _sensorID = 90393;
  int _cspin;
};

#endif /* ADAFRUIT_MLX90393_H */
