#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

void i2c_initialisation(i2c_inst_t *port,uint freq, int SDA_pin,int SCL_pin);

uint8_t wifi_chip_initialisation(void);