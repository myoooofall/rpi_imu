#include "device_CM4.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) {
    motors_device(num, i2c_addr_t);

    // shoot
    pinMode(GPIO_CHARGE, OUTPUT);
    pinMode(PWM0_SHOOT, PWM_OUTPUT);
    pwmSetClock(320);
    // infrare
    pinMode(GPIO_INFRARE_OUT, INPUT);
    pinMode(GPIO_INFRARE_IN, OUTPUT);
}

void devicez::motors_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;
    if (wiringPiSetup() != 0) {
        std::cout << "wiringPi error!" << std::endl;
    }
    for (int i=0; i<device_num; i++) {
        if (i2c_addr_t)   i2c_addr[i] = i2c_addr_t[i];
        motors_i2c[i] = wiringPiI2CSetup(i2c_addr[i]);
        // std::cout << i << ": " << std::hex << i2c_addr[i] << " ";
    }
}

int devicez::motors_detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        Rx_buf[i] = wiringPiI2CRead(motors_i2c[i]); // TODO: read specific bits
        if (Rx_buf[i] != -1)    motors_on++;
    }
    if (i2c_testmode && motors_on) {
        // Output
        std::cout << "huibao: ";
        for (int i=0; i<device_num; i++)    std::cout<< Rx_buf[i] << " ";
        std::cout << std::endl;
    }
    return motors_on;
}

void devicez::motors_write(int* vel_pack) {
    for (int i=0; i<device_num; i++) {
        wiringPiI2CWrite(motors_i2c[i], abs(vel_pack[i]));
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    // Output
    if (i2c_testmode) {
        std::cout << "motor speed: ";
        for (int i=0; i<device_num; i++)    std::cout<< abs(vel_pack[i]) << " ";
        std::cout << std::endl;
    }
}

uint8_t devicez::shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    pwmWrite(PWM0_SHOOT, 0);

    if(test_charge_count++ > 1000) {
        digitalWrite(GPIO_CHARGE, HIGH);    // start charge
        std::cout << "Robot_Is_Boot_charged" << std::endl;
        test_charge_count = 0;
        Robot_Is_Boot_charged = 1;          // enable shoot
    }else if(test_charge_count == 20) {
        digitalWrite(GPIO_CHARGE, LOW);    // stop charge
        pwmWrite(PWM0_SHOOT, 0);
    }
    
    if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
        digitalWrite(GPIO_CHARGE, LOW);    // stop charge
        pwmWrite(PWM0_SHOOT, Robot_Boot_Power*3);
        Robot_Is_Boot_charged = 0;
        test_charge_count = 0;
        std::cout << "shoot" << std::endl;
    }
    return Robot_Is_Boot_charged;
}

void devicez::infrare_detect() {}
void devicez::dribbler() {}