#ifndef I2C_H
#define I2C_H

#include "device.h"
#include <pigpio.h>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"
// using BCM number
#define GPIO_SHOOT          23
#define GPIO_CHIP           24
#define GPIO_CHARGE         17
#define GPIO_BUZZER         12

class devicez : public device{
public:
    devicez(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(std::vector<int>& vel_pack) override;
    void motors_write_single(int motor_id, int vel) override;
    uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) override;
    void infrare_detect() override;
    void dribbler(int dribble_val) override;

    void buzzer_once(int freq) override;
    void buzzer_start();

    void charge_switch();
    int adc_infrare();
    float adc_cap_vol();
    float adc_bat_vol();

    void write_uart(uint8_t* buff);
    void read_uart(uint8_t* buff);

    void adc_switch(int control_byte);
    std::vector<int> get_encoder();
    std::vector<int> get_pid();

    void motors_write_pid(std::vector<int>& pid_pack);
    // void motors_pid_write_single(int motor_id, char* pid_pack);

    // TODO: i2c-read-write any size devices/buffer

    ~devicez() {
        gpioTerminate();
    }

private:
    std::mutex mutex_i2c;
    std::mutex mutex_uart;
    std::vector<std::jthread> i2c_th_single;
    int motors_i2c_handle[MAX_MOTOR];
    int dribbler_i2c_handle;
    int adc_i2c_handle;

    mraa::Uart* uart;

    uint8_t motors_i2c_addr[4];
};

#endif