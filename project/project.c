    #include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "initialise_functions.h"


int main() {
    stdio_init_all();

    sleep_ms(10000);
    i2c_initialisation(i2c0,400*1000,I2C_SDA,I2C_SCL);

    wifi_chip_initialisation();

    printf("Hello, world! \n");

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);  
        sleep_ms(250);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        sleep_ms(500);

    }
}