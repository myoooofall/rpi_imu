#ifndef I2C_H
#define I2C_H

#include "device.h"
#include <pigpio.h>
// using BCM number
#define GPIO_SHOOT          23
#define GPIO_CHIP           24
#define GPIO_CHARGE         17

class devicez : public device{
public:
    devicez(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(std::vector<int>& vel_pack) override;
    void motors_write_single(int motor_id, int vel) override;
    uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) override;
    void infrare_detect() override;
    void dribbler() override;

    void charge_switch();
    uint8_t shoot_test(uint8_t Robot_Boot_Power);
    int adc_infrare();
    float adc_cap_vol();

private:
    std::vector<std::jthread> i2c_th_single;
    int motors_i2c_handle[MAX_MOTOR];
    int adc_i2c_handle;

    uint8_t motors_i2c_addr[4];
};

#endif