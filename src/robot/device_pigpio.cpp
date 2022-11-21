#include "device_pigpio.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : i2c_th_single(num) {
    
    if (gpioInitialise() < 0) {
        zos::error("pigpio error!\n");
    }
    motors_device(num, i2c_addr_t);
    adc_i2c_handle = i2cOpen(config::i2c_bus, config::adc_addr, 0);
    i2cWriteByte(adc_i2c_handle, 0x40);

    // shoot
    gpioSetMode(GPIO_SHOOT, PI_OUTPUT);
    gpioSetMode(GPIO_CHIP, PI_OUTPUT);
    gpioWrite(GPIO_SHOOT, PI_LOW);
    gpioWrite(GPIO_CHIP, PI_LOW);
    // pwmSetClock(320);
    // infrare
    // pinMode(GPIO_INFRARE_OUT, INPUT);
    // pinMode(GPIO_INFRARE_IN, OUTPUT);
}

void devicez::motors_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;

    if(i2c_addr_t)   std::copy(i2c_addr_t, i2c_addr_t+device_num, motors_i2c_addr);
    else {
        std::copy(config::motors_addr, config::motors_addr+device_num, motors_i2c_addr);
        zos::info("use default i2c address\n");
    }
    
    for (int i=0; i<device_num; i++) {
        motors_i2c_handle[i] = i2cOpen(config::i2c_bus, motors_i2c_addr[i], 0);
        // std::cout << i << ": " << std::hex << i2c_addr[i] << " ";
    }
}

int devicez::motors_detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        vel_encoder[i] = i2cReadByte(motors_i2c_handle[i]); // TODO: read specific bits
        if (vel_encoder[i] != PI_I2C_READ_FAILED)    motors_on++;
    }
    if (i2c_testmode && motors_on) {
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
    return motors_on;
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        i2c_th_single[i] = std::jthread(&devicez::motors_write_single, this, i, abs(vel_pack[i]));
        // i2c_th_single[i].join();
    }
    // Output
    if (i2c_testmode) {
        zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
}

void devicez::motors_write_single(int motor_id, int vel) {
    i2cWriteByte(motors_i2c_handle[motor_id], abs(vel));
    // zos::log("motor id: {}, vel: {}\n", motor_id, vel);
    vel_encoder[motor_id] = i2cReadByte(motors_i2c_handle[motor_id]); // TODO: read specific bits
}

uint8_t devicez::shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    // pwmWrite(PWM0_SHOOT, 0);

    // if(test_charge_count++ > 1000) {
    //     digitalWrite(GPIO_CHARGE, HIGH);    // start charge
    //     zos::info("Robot is boot charged\n");
    //     test_charge_count = 0;
    //     Robot_Is_Boot_charged = 1;          // enable shoot
    // }else if(test_charge_count == 20) {
    //     digitalWrite(GPIO_CHARGE, LOW);    // stop charge
    //     pwmWrite(PWM0_SHOOT, 0);
    // }
    
    // if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
    //     digitalWrite(GPIO_CHARGE, LOW);    // stop charge
    //     pwmWrite(PWM0_SHOOT, Robot_Boot_Power*3);
    //     Robot_Is_Boot_charged = 0;
    //     test_charge_count = 0;
    //     zos::info("shoot");
    // }
    return Robot_Is_Boot_charged;
}

void devicez::charge_switch() {
    float boot_cap_vol = adc_cap_vol();
    if (boot_cap_vol > 200) {
        gpioWrite(GPIO_CHARGE, PI_LOW);
        zos::status("stop charge (now {}V)\n", boot_cap_vol);
    }else if (boot_cap_vol < 100) {
        gpioWrite(GPIO_CHARGE, PI_HIGH);
        // zos::status("start charge\n");
    }
}

uint8_t devicez::shoot_test(uint8_t Robot_Boot_Power) {
    if(Robot_Boot_Power > 0 && adc_cap_vol() > 60) {
        std::chrono::duration<int, std::nano> _step = std::chrono::milliseconds(1);
    
        gpioWrite(GPIO_SHOOT, PI_HIGH);
        zos::log("shoot: {}     ", Robot_Boot_Power);
        std::this_thread::sleep_for(_step*Robot_Boot_Power);
        gpioWrite(GPIO_SHOOT, PI_LOW);
        zos::status("vol remain: {}\n", adc_cap_vol());
    }else {
        zos::log("low voltage: {}, boot power: {}\n", adc_cap_vol(), Robot_Boot_Power);
    }
    gpioWrite(GPIO_SHOOT, PI_LOW);
    return 0;
}

int devicez::adc_infrare() {
    i2cWriteByte(adc_i2c_handle, 0x41);

    adc_val = i2cReadByte(adc_i2c_handle);
    zos::status("adc value: {}\n", adc_val);
    return adc_val;
}

float devicez::adc_cap_vol() {
    i2cWriteByte(adc_i2c_handle, 0x40);
    adc_val = i2cReadByte(adc_i2c_handle);
    float cap_vol = adc_val * config::adc_cap_vol_k;
    // float cap_vol = adc_val;
    // zos::status("cap voltage: {}\n", cap_vol);
    return cap_vol;
}

void devicez::infrare_detect() {}
void devicez::dribbler() {}