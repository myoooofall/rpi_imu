#include "device_CM4.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : i2c_th(num) {
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

    if(i2c_addr_t)   std::copy(i2c_addr_t, i2c_addr_t+device_num, motors_i2c);
    else {
        std::copy(config::motors_addr, config::motors_addr+device_num, motors_i2c);
        zos::info("use default i2c address\n");
    }
    
    for (int i=0; i<device_num; i++) {
        motors_i2c[i] = wiringPiI2CSetup(motors_i2c[i]);
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
        // std::cout << "huibao: ";
        // for (int i=0; i<device_num; i++)    std::cout<< Rx_buf[i] << " ";
        // std::cout << std::endl;
        // std::string motor_str;
        zos::log("motor status: {}\n", fmt::ptr(&Rx_buf));
    }
    return motors_on;
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        i2c_th.enqueue(&devicez::motors_write_single, this, i, abs(vel_pack[i]));
    }
    // Output
    if (i2c_testmode) {
        zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
    }
}

void devicez::motors_write_single(int motor_id, int vel) {
    wiringPiI2CWrite(motors_i2c[motor_id], abs(vel));
}

uint8_t devicez::shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    pwmWrite(PWM0_SHOOT, 0);

    if(test_charge_count++ > 1000) {
        digitalWrite(GPIO_CHARGE, HIGH);    // start charge
        zos::info("Robot is boot charged\n");
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
        zos::info("shoot");
    }
    return Robot_Is_Boot_charged;
}

void devicez::infrare_detect() {}
void devicez::dribbler() {}