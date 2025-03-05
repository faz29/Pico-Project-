#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>
#include "initialise_functions.h"

//====================================================================================================
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};


void i2c_initialisation(i2c_inst_t *port,uint freq){
// I2C Initialisation. Using it at 400Khz.
    i2c_init(port, freq);
    // gpio_set_function(DPS310_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(DPS310_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(DPS310_SDA);
    // gpio_pull_up(DPS310_SCL);

    gpio_set_function(MPU6050_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA);
    gpio_pull_up(MPU6050_SCL);


int P;
    if (port==i2c0){
       P = 0;
    }
    if (port==i2c1){
       P = 1;
    }

    printf("I2C Initialised at %dkHz on port %p !!\n", freq/1000, port);

}

uint8_t wifi_chip_initialisation(void){
        // Initialise the Wi-Fi chip
    if (cyw43_arch_init()==-1) {
        printf("Wi-Fi init failed\n");
        return -1;
    }   
    else{
        printf("Wifi chip initialised !\n");
    }
}

void uart_initialisation(uart_inst_t*uart_port,int uart_Brate, int tx_pin, int rx_pin, int data_bits, int stop_bits){    
    
    //set up the UART TX and RX pins
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart_port,tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart_port,rx_pin));    

    
    uart_set_hw_flow(uart_port, false, false);
    uart_set_format(uart_port, data_bits, stop_bits, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_port, false);

    uart_init(uart_port, uart_Brate);
    printf("UART initialised at %d baud rate with TX on pin %d and RX on pin %d !\n", uart_Brate, tx_pin, rx_pin);
}

void pwm_initialisation(int pwm_pin,uint chan,int pulse_width,int ledPin){
    gpio_set_function(pwm_pin,GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pwm_pin);

    //clk div of 125 gives a frequency of 1MHz meaning a per tick width of 1us.
    pwm_set_clkdiv(slice, 125);
    pwm_set_wrap(slice, 1000);
    pwm_set_chan_level(slice,chan,pulse_width); //maybe moved to main loop
    pwm_set_enabled(slice,true);    
    printf("PWM initialised on pin %d !\n",pwm_pin);

    sleep_ms(2000);
    // gpio_init(ledPin);
    // gpio_set_dir(ledPin,GPIO_OUT);
    // gpio_put(ledPin,1);
}

void esc_calibration(int pwm_pin,uint chan,int ledPin, int arm_sleep){
    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    //arm sequence: zero throttle (125us), small delay, then full (250us), small delay then zero throttle.
    pwm_set_chan_level(slice,chan,125); 
    sleep_ms(arm_sleep); 
    pwm_set_chan_level(slice,chan,250);
    sleep_ms(arm_sleep);
    pwm_set_chan_level(slice,chan,125); 

    //turn on Led when arming is complete
    gpio_init(ledPin);
    gpio_set_dir(ledPin,GPIO_OUT);
    gpio_put(ledPin,1);
    printf("Arm sequence complete!\n");
}

void led_on(int ledPin, bool state){
    gpio_init(ledPin);
    gpio_set_dir(ledPin,GPIO_OUT);
    gpio_put(ledPin,state);

}

void read_throttle(int pwm_pin, int* c,uint16_t* throttleVal){
    uint8_t buffer[1];
    uint8_t throttleArray[5];
    uint32_t joined;
    //uint16_t throttleVal;
    uint slice = pwm_gpio_to_slice_num(pwm_pin);

    //pwm_set_chan_level(slice,0,140);
    if (uart_is_readable(uart0) == true){
        while (true) {
            led_on(7,true);
            uart_read_blocking(uart0, buffer, 1);
            if (buffer[0] == 0xFF) {
                break;
            }
        }

        uart_read_blocking(uart0, throttleArray, 4);
    led_on(7,false);
    buffer[0]=0;
    joined = 0;

    for (int j = 0; j<4; j++){
        //printf("%d ",throttleArray[j]);
        joined += (throttleArray[j]<<(8*(3-j))); 
        }
    
    *throttleVal = 125 + (125*joined)/1020;

    pwm_set_chan_level(slice,0,*throttleVal);
    *c = joined;
    }
}

void read_brake(int pwm_pin, int* d,uint16_t* brakeVal){
    //uint8_t buffer[1];
    uint8_t brakeArray[4];
    uint32_t joined;
    //uint16_t throttleVal;
    uint slice = pwm_gpio_to_slice_num(pwm_pin);


//  if (uart_is_readable(uart0) == true){
//         while (true) {
//             led_on(7,true);
//             uart_read_blocking(uart0, buffer, 1);
//             if (buffer[0] == 0xFE) {
//                 break;
//             }
//         }

//     uart_read_blocking(uart0, brakeArray, 4);
//     // led_on(7,false);
//     buffer[0]=0;


//    }
   joined = 0;

   for (int j = 0; j<4; j++){
       //printf("%d ",brakeArray[j]);
       joined += (brakeArray[j]<<(8*(3-j))); 
       }

   printf("\n");
   
   printf("\nBrake: %d\n\n",joined);
   
   *brakeVal = 125 + (125*joined)/1020;

   //    pwm_set_chan_level(slice,0,*brakeVal);

   *d = joined;

}

void read_controller(uart_inst_t* uart_port,int pwm_pin, int* t,int* b,uint16_t* throttle, uint16_t* brake){    

    uint8_t check[1];
    uint8_t controllerArray[9];
    uint32_t brakeJoined;
    uint32_t throttleJoined;
    uint slice = pwm_gpio_to_slice_num(pwm_pin);

    if (uart_is_readable(uart_port) == true){
        // while (true) {
            led_on(7,true);
            uart_read_blocking(uart_port, check, 1);
            if (check[0] == 0xFF) {

    //    }

        uart_read_blocking(uart_port, controllerArray, 8);
    check[0]= throttleJoined = 0;

    for (int j = 0; j<4; j++){
        throttleJoined += (controllerArray[j]<<(8*(3-j))); 
        }
    for (int j = 4; j<8; j++){
        brakeJoined += (controllerArray[j]<<(8*(7-j)));
    }
    
    *throttle = 125 + (125*throttleJoined)/1020;

    *brake = 125 + (125*brakeJoined)/1020;

    // pwm_set_chan_level(slice,0,*throttle);

    *t = throttleJoined;
    *b = brakeJoined;    
}
}
    
}

void PID(float* E, float*sp, float* pv, float Kp, float Ki, float Kd,int* maxstep,float* pulse){
    //*E = *pv-*sp;
    //*E = *sp - *pv;
    float PID,P,D,dE,dt;
    static float I = 0;
    static float prevE = 0;
    static uint64_t prevTime = 0;

    if(*sp>100 || *sp<-100){
        *sp = 0;}
    if (*sp>50 && *sp<100){ 
        *sp =50;}
    if(*sp<-50 && *sp>-100){
        *sp = -50;}
    
    uint64_t currTime = time_us_64();

    dt = (currTime-prevTime)/1000000.0;
    if (dt < 0.000001) {dt = 0.000001;}   
    dE = *E - prevE;

    P = Kp*(*E);

    if(*pulse>9 && *pulse<251){
    I += Ki*(*E)*dt;
    }
    
    D = dE/dt;      
    PID = P + I + Kd*D;
    
    if(PID>*maxstep){
        PID = *maxstep;}
    if(PID<-(*maxstep)){
        PID= -(*maxstep);}

    *pulse += PID;
    if(*pulse>125){
        *pulse = 125;
    }
    if(*pulse<20){
        *pulse = 20;
    }

    prevTime = currTime;
    prevE = *E;
}

uint8_t MPU6050_Init(i2c_inst_t *i2cPort) {
    uint8_t check;
    uint8_t Data[2];
    uint8_t who = WHO_AM_I_REG;

    // check device ID WHO_AM_I
    
    i2c_write_blocking(i2cPort,MPU6050_ADDR,&who,1,true);
    i2c_read_blocking(i2cPort,MPU6050_ADDR,&check,1,false);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data[0] = PWR_MGMT_1_REG;
        Data[1] = 0;

        i2c_write_blocking(i2cPort,MPU6050_ADDR,Data,2,false);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data[0] = SMPLRT_DIV_REG;
        Data[1] = 0x07;

        i2c_write_blocking(i2cPort,MPU6050_ADDR,Data,2,false);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->   2g
        Data[0] = ACCEL_CONFIG_REG;
        Data[1] = 0x00;    

        i2c_write_blocking(i2cPort,MPU6050_ADDR,Data,2,false);
        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->   250  /s
        Data[0] = GYRO_CONFIG_REG;
        Data[1] = 0x00;
        i2c_write_blocking(i2cPort,MPU6050_ADDR,Data,2,false);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(i2c_inst_t *i2cPort, MPU6050_t *DataStruct){

    uint8_t Rec_Data[6];
    
    uint8_t addr[1] = {ACCEL_XOUT_H_REG};
    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    i2c_write_blocking(i2cPort,MPU6050_ADDR,addr,1,true);
    i2c_read_blocking(i2cPort,MPU6050_ADDR,Rec_Data,6,false);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(i2c_inst_t *i2cPort, MPU6050_t *DataStruct){

    uint8_t Rec_Data[6];
    uint8_t addr[1] = {GYRO_XOUT_H_REG};

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    //HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    i2c_write_blocking(i2cPort,MPU6050_ADDR,addr,1,true);
    i2c_read_blocking(i2cPort,MPU6050_ADDR,Rec_Data,6,false);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps ( /s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;


}

void MPU6050_Read_Temp(i2c_inst_t *i2cPort, MPU6050_t *DataStruct){

    uint8_t Rec_Data[2];
    int16_t temp;
    uint8_t addr[1] = {TEMP_OUT_H_REG};

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    // HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    i2c_write_blocking(i2cPort,MPU6050_ADDR,addr,1,true);
    i2c_read_blocking(i2cPort,MPU6050_ADDR,Rec_Data,2,false);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

}

void MPU6050_Read_All(i2c_inst_t *i2cPort, MPU6050_t *DataStruct){

    uint8_t Rec_Data[14];
    int16_t temp;
    uint8_t addr[1] = {ACCEL_XOUT_H_REG};

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    // HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    i2c_write_blocking(i2cPort,MPU6050_ADDR,addr,1,true);
    i2c_read_blocking(i2cPort,MPU6050_ADDR,Rec_Data,14,false);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    //double dt = (double) (HAL_GetTick() - timer) / 1000;
    //for STM MCUs hal_gettick() returns the time in milliseconds as a 32 bit unsigned int
    //with a pico, using time_us_32() returns time in as uint32_t microseconds however,
    //so it needs to be divided by 1E6 to return the same value 

    double dt = (double) (time_us_32()-timer)/1E6;
    timer = time_us_32();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);


}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt){


    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;

}
