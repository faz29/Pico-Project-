#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "mpu6050.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21


float AngX, AngY;
MPU6050_t MPU6050;

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    gpio_init(6);
    gpio_set_dir(6,true);
    gpio_put(6,0);

    MPU6050_Init(I2C_PORT);

    uint32_t wait = time_us_32();
    uint32_t prev_wait = 0;
    int interval = 5E5;

    printf("Hello, world!\n");
    
    while (true) {
        
        gpio_put(6,1);
        MPU6050_Read_All(I2C_PORT,&MPU6050);
            AngX = MPU6050.KalmanAngleX;
            AngY = MPU6050.KalmanAngleY;

        if(wait-prev_wait >= interval){
            printf("\nX: %f  Y: %f \n",AngX,AngY);
            prev_wait = wait;
        } 
        wait = time_us_32();

    }
    
}
