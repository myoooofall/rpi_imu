#ifndef DEVICE_H
#define DEVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>

#define MAX_MOTOR   4

class device{
public:
    virtual void motors_device(int num, uint8_t *i2c_addr_t) = 0;
    virtual int motors_detect() = 0;
    virtual void motors_write(int* vel_pack) = 0;
    virtual uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) = 0;
    virtual void infrare_detect() = 0;
    virtual void dribbler() = 0;
    
    void output_test() {
        i2c_testmode = true;
    };

protected:
    int device_num = MAX_MOTOR;
    uint8_t i2c_addr[MAX_MOTOR];
    int Rx_buf[MAX_MOTOR];
    bool i2c_testmode = false;

    int test_charge_count = 0;

private:
};

#endif