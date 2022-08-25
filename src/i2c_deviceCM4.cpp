#include "i2c_deviceCM4.h"

i2c_device::i2c_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;
    if (wiringPiSetup() != 0) {
        std::cout << "wiringPi error!" << std::endl;
    }
    for (int i=0; i<device_num; i++) {
        if (i2c_addr_t)   i2c_addr[i] = i2c_addr_t[i];
        device[i] = wiringPiI2CSetup(i2c_addr[i]);
        // std::cout << i << ": " << std::hex << i2c_addr[i] << " ";
    }
}

int i2c_device::detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        Rx_buf[i] = wiringPiI2CRead(device[i]); // TODO: read specific bits
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

void i2c_device::i2c_write(int* vel_pack) {
    for (int i=0; i<device_num; i++) {
        wiringPiI2CWrite(device[i], abs(vel_pack[i]));
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    // Output
    if (i2c_testmode)
    {
        std::cout << "motor speed: ";
        for (int i=0; i<device_num; i++)    std::cout<< abs(vel_pack[i]) << " ";
        std::cout << std::endl;
    }
}

void i2c_device::output_test() {
    i2c_testmode = true;
}
