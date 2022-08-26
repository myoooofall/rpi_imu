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
#include "device.h"

#define GPIO_INFRARE_IN     4
#define GPIO_INFRARE_OUT    5
#define PWM0_SHOOT          26
#define GPIO_CHARGE         0

class i2c_device : public device{
public:
    i2c_device(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(int* vel_pack) override;
    uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) override;
    void infrare_detect() override;
    void dribbler() override;
};

#endif