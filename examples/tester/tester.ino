#include <Wire.h>

#include "Adafruit_MLX90393.h"

/* Pinout based on M0 Basic Feather */
#define INT_PIN         (5)   /* Pin D2 is interrupt 0 on the 328 */
#define TRIG_PIN        (6)   /* Pin D3 is interrupt 1 on the 328 */
#define SENB_PIN        (9)   /* AKA MISO */
#define POWER_PIN       (10)  /* Power the board via GPIO since we don't have reset */

Adafruit_MLX90393 sensor = Adafruit_MLX90393(393);
uint32_t g_int_times_fired = 0;
uint32_t g_trig_times_fired = 0;

void int_isr_handler(void)
{
  /* Increment the INT count every time this interrupt fires. */
  g_int_times_fired++;
}

void trig_isr_handler(void)
{
  /* Increment the TRIG count every time this interrupt fires. */
  g_trig_times_fired++;
}

void print_test(const char *str)
{
    size_t l = strlen(str);
    Serial.print(str);
    for (size_t i = 68-l; i != 0; i--) {
        Serial.print(".");
    }
}

/**
 * Prints the results of the test, and on ERR (status = false)
 * halts program execution in an infinite error loop.
 */
void print_result(bool status)
{
    if (status) {
        Serial.println("OK");
    } else {
        Serial.println("ERROR!");
        while(1) {
            /* ToDo: Audible beep in addition to ERROR blinky */
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
}

void test_read_data(void)
{
  float x, y, z;

  print_test("Reading sensor data");
  print_result(sensor.readData(&x, &y, &z));

  Serial.print("  X: "); Serial.print(x, 4); Serial.println(" uT");
  Serial.print("  Y: "); Serial.print(y, 4); Serial.println(" uT");
  Serial.print("  Z: "); Serial.print(z, 4); Serial.println(" uT");
}

void test_int_pin(void)
{
  float x, y, z;
  int int_count_prev = g_int_times_fired;

  print_test("Testing INT pin");

  /* Request a new data sample, which should cause the INT to fire
   * when a new (replacement) data sample is available. */
  sensor.readData(&x, &y, &z);
  sensor.readData(&x, &y, &z);
  sensor.readData(&x, &y, &z);

  for (uint32_t c = 0; c < 1000; c++) {
    if (g_int_times_fired > int_count_prev) {
      print_result(true);
      Serial.print("  Times INT has fired: ");
      Serial.println(g_int_times_fired);
      return;
    }
  }

  /* Timed out waiting for INT to change. */
  print_result(false);
}

void test_trig_pin(void)
{
  float x, y, z;
  int trig_count_prev = g_trig_times_fired;

  print_test("Testing TRIG pin");

  /* Enable INT on the TRIG_INT pin (instead of default TRIG) */
  sensor.setTrigInt(true);

  /* Request a new data sample, which should cause the TRIG to fire
   * when a new (replacement) data sample is available. */
  sensor.readData(&x, &y, &z);
  sensor.readData(&x, &y, &z);
  sensor.readData(&x, &y, &z);

  for (uint32_t c = 0; c < 1000; c++) {
    if (g_trig_times_fired > trig_count_prev) {
      print_result(true);
      Serial.print("  Times TRIG has fired: ");
      Serial.println(g_trig_times_fired);
      return;
    }
  }

  /* Timed out waiting for TRIG to change. */
  print_result(false);
}

void test_senb_pin(void)
{
  print_test("Testing SENB pin");

  /* Test value of SENB 10 times with a delay. If the value is
   *  consistently high, it means there is a reliable connection
   *  and the pin isn't 'floating'.
   */
  for (uint8_t t = 0; t < 10; t++) {
    int val = digitalRead(SENB_PIN);
    if (!val) {
      print_result(false);
      return;
    }
    delay(10);
  }

  /* Timed out waiting for TRIG to change. */
  print_result(true);
}

void reset_sensor(void)
{
  digitalWrite(POWER_PIN, LOW);
  delay(200);
  digitalWrite(POWER_PIN, HIGH);
  delay(200);
}

void test_spi_mode(void)
{
  pinMode(SENB_PIN, OUTPUT);

  print_test("Resetting into SPI mode via SENB/CS");
  digitalWrite(SENB_PIN, LOW);  /* SPI mode */
  reset_sensor();
  print_result(true);

  print_test("Checking we're no longer in I2C mode");
  uint8_t result = sensor.begin();
  print_result(result ? false : true);

  print_test("Resetting back into I2C mode");
  digitalWrite(SENB_PIN, HIGH); /* I2C mode */
  reset_sensor();
  print_result(sensor.begin());

  pinMode(SENB_PIN, INPUT);
}

void setup(void)
{
  Serial.begin(9600);

  /* Wait for serial on USB platforms. */
  while(!Serial);

  /* Setup LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Setup input pins and attach interrupts. */
  pinMode(TRIG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIG_PIN), trig_isr_handler, RISING);
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), int_isr_handler, RISING);
  pinMode(SENB_PIN, INPUT);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  Serial.println("Starting Adafruit MLX90393 Demo\n");

  reset_sensor();

  /* Init test */
  print_test("Checking for MLX90393");
  print_result(sensor.begin());

  /* Test Suite */
  test_read_data();
  test_int_pin();
  test_trig_pin();
  test_senb_pin();
  test_spi_mode();

  Serial.println("\nDONE! All tests OK!");
}

void loop(void)
{
    delay(500);
}
