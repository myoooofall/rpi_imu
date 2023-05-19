#include "device_pigpio.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : i2c_th_single(num) {
    
    if (gpioInitialise() < 0) {
        zos::error("pigpio error!\n");
    }
    motors_device(num, i2c_addr_t);
    dribbler_i2c_handle = i2cOpen(config::i2c_bus, config::dribbler_addr, 0);
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
    // buzzer
    gpioSetMode(GPIO_BUZZER, PI_OUTPUT);
    buzzer_start();

    try {
        uart = new mraa::Uart("/dev/ttyAMA1");
    } catch (std::exception& e) {
        std::cerr << "Error while setting up raw UART, do you have a uart?" << std::endl;
        std::terminate();
    }
    if (uart->setBaudRate(9600) != mraa::SUCCESS) {
        std::cerr << "Error setting parity on UART" << std::endl;
    }
    for (int i=0; i<10; i++) {
        read_uart(NULL);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void devicez::buzzer_start() {
    buzzer_once(500);
    buzzer_once(1600);
}
void devicez::buzzer_once(int freq) {
    gpioSetPWMfrequency(GPIO_BUZZER, freq);
    gpioPWM(GPIO_BUZZER, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    gpioPWM(GPIO_BUZZER, 0);
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
        // FIXME: i2c read
        vel_encoder[i] = i2cReadByte(motors_i2c_handle[i]); // TODO: read specific bits
        if (vel_encoder[i] != PI_I2C_READ_FAILED)    motors_on++;
    }
    // FIXME: i2c read
    // device_num = motors_on;
    if (device_num && i2c_testmode) {
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
    return motors_on;
}

void devicez::dribbler(int dribble_val) {
    // int dribble_val = (0x0c) | (0x03);
    std::scoped_lock lock(mutex_i2c);
    // zos::log("dribble val: {:#04x}\n", dribble_val);
    i2cWriteByte(dribbler_i2c_handle, dribble_val);
    // i2cWriteDevice(dribbler_i2c_handle, (char*)dribble_val, 1);
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        i2c_th_single[i] = std::jthread(&devicez::motors_write_single, this, i, vel_pack[i]);
        // i2c_th_single[i].join();
    }
    // Output
    if (device_num && i2c_testmode) {
        zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
}

void devicez::motors_write_single(int motor_id, int vel) {
    uint8_t vel_pack[3] = {0x0};
    uint8_t encoder_pack[3] = {0x0};
    vel_pack[0] = 0xfa;
    vel_pack[1] = ((abs(vel) >> 8) & 0x1f) | (((vel>=0)?0:1) << 5) | (0x01 << 6);
    vel_pack[2] = (abs(vel) & 0xff);
    {
        std::scoped_lock lock(mutex_i2c);
        i2cWriteDevice(motors_i2c_handle[motor_id], (char*)vel_pack, 3);
        // zos::log("motor id: {}, vel_int: {}, vel_pack: {:#04x} {:#04x} {:#04x}\n", motor_id, vel, (char)vel_pack[0], (char)vel_pack[1], (char)vel_pack[2]);
        i2cReadDevice(motors_i2c_handle[motor_id], (char*)encoder_pack, 3);
        // zos::warning("encoder pack: {} {} {}\n", encoder_pack[0], encoder_pack[1], encoder_pack[2]);
    }
    
    if (encoder_pack[0] == 0xfa) {
        int vel_temp = ((encoder_pack[1] & 0x1f) << 8) + encoder_pack[2];
        vel_encoder[motor_id] = ((encoder_pack[1] & 0x20)>>5)?(-vel_temp):vel_temp;
        // zos::status("encoder id: {}, vel_encoder: {}\n", motor_id, vel_encoder[motor_id]);
    }else {
        vel_encoder[motor_id] = 0;
        // zos::warning("wrong pack: {} {} {}\n", encoder_pack[0], encoder_pack[1], encoder_pack[2]);
    }
    // i2cWriteByte(motors_i2c_handle[motor_id], abs(vel));
    // zos::log("motor id: {}, vel_int: {}\n", motor_id, vel);
    // encoder_read_single(motor_id);
    // vel_encoder[motor_id] = i2cReadByte(motors_i2c_handle[motor_id]); // TODO: read specific bits
    // zos::status("motor id: {}, vel_encoder: {}\n", motor_id, vel_encoder[motor_id]);
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

uint8_t devicez::shoot_chip(uint8_t Robot_Chip_Or_Shoot, uint8_t Robot_Boot_Power) {
    std::jthread th_shoot([this, Robot_Chip_Or_Shoot, Robot_Boot_Power] {
        int kick_gpio;
        if(Robot_Chip_Or_Shoot == 0) {
            kick_gpio = GPIO_SHOOT;
        }else {
            kick_gpio = GPIO_CHIP;
        }

        if(Robot_Boot_Power > 0 && adc_cap_vol() > 100) {
            std::chrono::duration<int, std::nano> _step = std::chrono::microseconds(80);
        
            gpioWrite(kick_gpio, PI_HIGH);
            // zos::log("shoot: {}     ", _step*Robot_Boot_Power);
            std::this_thread::sleep_for(_step*Robot_Boot_Power);
            gpioWrite(kick_gpio, PI_LOW);
            zos::status("vol remain: {}\n", adc_cap_vol());
        }else {
            zos::log("low voltage: {}, boot power: {}\n", adc_cap_vol(), Robot_Boot_Power);
        }
        gpioWrite(kick_gpio, PI_LOW);
    });
    th_shoot.detach();
    return 0;
}

void devicez::adc_switch(int control_byte) {
    std::jthread i2c_th([this, control_byte] {
        // std::scoped_lock lock(mutex_i2c);
        i2cWriteByte(adc_i2c_handle, control_byte);
    });
}

int devicez::adc_infrare() {
    std::scoped_lock lock(mutex_i2c);
    adc_switch(0x40);
    adc_val = i2cReadByte(adc_i2c_handle);
    adc_val = i2cReadByte(adc_i2c_handle);
    // zos::status("adc value: {}\n", adc_val);
    return adc_val/130;
}

float devicez::adc_bat_vol() {
    std::scoped_lock lock(mutex_i2c);
    adc_switch(0x42);
    adc_val = i2cReadByte(adc_i2c_handle);
    adc_val = i2cReadByte(adc_i2c_handle);
    // float cap_vol = adc_val * config::adc_cap_vol_k;
    // float cap_vol = adc_val;
    // zos::info("battery voltage: {}\n", adc_val);
    return adc_val;
}

float devicez::adc_cap_vol() {
    std::scoped_lock lock(mutex_i2c);
    adc_switch(0x41);
    adc_val = i2cReadByte(adc_i2c_handle);
    adc_val = i2cReadByte(adc_i2c_handle);
    float cap_vol = adc_val * config::adc_cap_vol_k;
    // float cap_vol = adc_val;
    // zos::status("cap voltage: {}\n", cap_vol);
    return cap_vol;
}

std::vector<int> devicez::get_encoder() {
    std::scoped_lock lock(mutex_i2c);
    return vel_encoder;
}

std::vector<int> devicez::get_pid() {
    // std::scoped_lock lock(mutex_i2c);
    // return vel_encoder;
    char pid_req_pack_temp[3] = {0xf9};
    char pid_get_pack_temp[3] = {0x0};
    std::vector<int> pid_real_temp(8, 0x0);
    for (int motor_id=0; motor_id<device_num; motor_id++) {
        int try_count = 0;
        while (pid_get_pack_temp[0] != 0x95 && try_count++ < 5) {
            std::scoped_lock lock(mutex_i2c);
            i2cWriteDevice(motors_i2c_handle[motor_id], (char*)pid_req_pack_temp, 3);
            i2cReadDevice(motors_i2c_handle[motor_id], (char*)pid_get_pack_temp, 3);
        }
        if(pid_get_pack_temp[0] == 0x95) {
            pid_real_temp[motor_id*2] = pid_get_pack_temp[1];
            pid_real_temp[motor_id*2+1] = pid_get_pack_temp[2];
        }else {
            zos::warning("motor {} get pid failed\n", motor_id);
            pid_real_temp[motor_id*2] = 0;
            pid_real_temp[motor_id*2+1] = 0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return pid_real_temp;
}

void devicez::motors_write_pid(std::vector<int>& pid_pack) {
    char pid_pack_temp_long[8];
    std::copy(begin(pid_pack), end(pid_pack), pid_pack_temp_long);
    char pid_pack_temp[3] = {0xf5};
    // zos::log("pid: {:#04x} {:#04x} {:#04x}\n", pid_pack_temp[0], pid_pack_temp[1], pid_pack_temp[2]);
    // FIXME: i<device_num

    for (int motor_id=0; motor_id<device_num; motor_id++) {
        int try_count = 0;
        pid_pack_temp[0] = 0xf5;
        pid_pack_temp[1] = pid_pack_temp_long[motor_id*2];
        pid_pack_temp[2] = pid_pack_temp_long[motor_id*2+1];
        if ((pid_pack_temp[1]+pid_pack_temp[2]) == 0) {
            zos::log("motor {} pid is empty\n", motor_id);
            continue;
        }
        uint8_t pid_done_flag = 0x00;
        while (pid_done_flag != 0x59 && try_count++ < 10) {
            std::scoped_lock lock(mutex_i2c);
            i2cWriteDevice(motors_i2c_handle[motor_id], pid_pack_temp, 3);

            // char pid_report_temp[2] = {0x0};
            // i2cReadDevice(motors_i2c_handle[motor_id], (char*)pid_report_temp, 2);
            // zos::log("pid setting: {:#04x} {:#04x}\n", pid_report_temp[0], pid_report_temp[1]);

            pid_done_flag = i2cReadByte(motors_i2c_handle[motor_id]);
            // zos::log("motor {} pid setting {:#04x}\n", motor_id, pid_done_flag);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        if(try_count >= 10){
            zos::error("motor {} set pid failed\n", motor_id);
        }else {
            // zos::log("motor {} pid set already\n", motor_id);
        }
    }
}

// void devicez::motors_pid_write_single(int motor_id, char* pid_pack) {
//     std::scoped_lock lock(mutex_i2c);
//     do {
//         i2cWriteDevice(motors_i2c_handle[motor_id], pid_pack, 3);
//         std::this_thread::sleep_for(std::chrono::microseconds(100));
//     }while (i2cReadByte(motors_i2c_handle[motor_id]) != 0xcc);
//     zos::log("motor {} pid set already\n");
// }

void devicez::infrare_detect() {}

#define MAX_SIZE 25
void devicez::write_uart(uint8_t* buff) {
    std::string buff_str(buff, buff+MAX_SIZE);
    std::scoped_lock lock(mutex_uart);
    uart->writeStr(buff_str);
    std::cout << "receive pack: " << buff_str << std::endl;
}
void devicez::read_uart(uint8_t* buff) {
    std::scoped_lock lock(mutex_uart);
    std::string buff_str = uart->readStr(MAX_SIZE);
    std::cout << "read pack: " << buff_str << std::endl;
    if (buff != NULL) {
        std::copy(buff_str.begin(), buff_str.begin()+MAX_SIZE, buff);
    }
}
