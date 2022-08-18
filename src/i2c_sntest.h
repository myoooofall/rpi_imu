#ifndef I2C_H
#define I2C_H

#include "main.h"

#define MAX_MOTOR 4

class i2c_device{
public:
    i2c_device(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL, int i2c_bus=0);
    int detect();
    void i2c_write(int* vel_pack);
    void i2c_read(int* vel_encoder);

private:
    int device_num = 0;
    uint8_t i2c_addr[MAX_MOTOR] = {0x28,0x29,0x30,0x31};

    mraa::I2c device = mraa::I2c(0);
    std::vector<mraa::I2c> devices;

    int Rx_buf[MAX_MOTOR];
};

#endif