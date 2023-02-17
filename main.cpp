#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "include/ads1115.hpp"

#define SDA 14
#define SCL 15
#define addr 0x48 
#define I2C i2c1



int main(){
    stdio_init_all();
    ads ads1115(I2C,SDA,SCL,addr);
    ads1115.init();
    
    while(1){
        int adc = ads1115.adcReadSingleEnd(0);
        float voltage = ads1115.getVoltageSingleEnd(0);
        printf("adc: %d voltage: %.4f\n",adc,voltage);
        sleep_ms(2000);
    }
    

}