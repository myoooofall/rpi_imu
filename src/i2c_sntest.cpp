#include "i2c_sntest.h"

i2c_device::i2c_device(int num, uint8_t *i2c_addr_t, int i2c_bus) : devices(num, {i2c_bus}) {
    device_num = num;
    std::cout << num << " motors" << std::endl;
    for (int i=0; i<device_num; i++) {
        if (i2c_addr_t)   i2c_addr[i] = i2c_addr_t[i];
        device.address(i2c_addr[i]);
        devices[i] = device;
    }
}

int i2c_device::detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        Rx_buf[i] = devices[i].readByte();
        if (Rx_buf[i] != -1)    motors_on++;
    }
    if (motors_on) {
        // Output
        // std::cout << "huibao: ";
        // for (int i=0; i<device_num; i++)    std::cout<< Rx_buf[i] << " ";
        // std::cout << std::endl;
    }
    return motors_on;
}

void i2c_device::i2c_write(int* vel_pack) {
    for (int i=0; i<device_num; i++) {
        devices[i].writeByte(vel_pack[i]);
        // wiringPiI2CWrite(device[i], vel_pack[i]);
        // delay(5);    // FIX
    }
    // Output
    // std::cout << "motor speed: ";
    // for (int i=0; i<device_num; i++)    std::cout<< vel_pack[i] << " ";
    // std::cout << std::endl;
}

void i2c_device::i2c_read(int* vel_encoder) {
    // int Curr_Vel = -1;
    for (int i=0; i<device_num; i++) {
        vel_encoder[i] = devices[i].readByte();
        // delay(5);    // FIXME
    }
}
