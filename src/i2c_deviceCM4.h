#ifndef I2C_H
#define I2C_H

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include "interface.h"

class i2c_device{
public:
    i2c_device(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    int detect();
    void i2c_write(int* vel_pack);
    void output_test();

private:
    int device_num = 0;
    uint8_t i2c_addr[MAX_MOTOR] = {0x28,0x29,0x30,0x31};
    int device[MAX_MOTOR];
    int Rx_buf[MAX_MOTOR];
    bool i2c_testmode = false;
};

#endif