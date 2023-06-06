#ifndef ROBOT_H
#define ROBOT_H

#include <thread>

// #ifdef OLD_VERSION
#include "nrf2401.h"
// #else
#include "wifiz.h"
// #endif

#include "robot_comm.pb.h"
#include "controlal.h"
// #include "controlal.h"

#ifdef ROCKPIS_VERSION
    #include "device_ROCKS.h"
#elif defined(CM4_VERSION)
    // #include "device_CM4.h"
    #include "device_pigpio.h"
#endif

#include <yaml-cpp/yaml.h>

class robotz {
public:
    robotz(int motor_num=4);
    ~robotz() = default;
    uint8_t robot_num = config::robot_id;
    devicez gpio_devices;
    // #ifdef OLD_VERSION
    comm_2401 nrf2401;
    // #else
    wifi_comm wifiz;
    // #endif
    void _wifi_cb(const asio::ip::udp::endpoint&,const void*,size_t);

    // controlal bangbang;
    void testmode_on();

    // uint8_t Robot_Is_Infrared;      //红外触发
    // uint8_t Robot_Is_Boot_charged;  //电容充电到60V
    uint8_t Robot_drib;
    uint8_t Robot_Chip_Or_Shoot;    //chip:1  shoot:0
    uint8_t Robot_Boot_Power = 0;
    uint8_t Robot_Is_Report;

    int16_t Vx_package = 0, Vy_package = 0, Vr_package = 0;  //下发机器人速度
    int16_t Vx_package_last = 0, Vy_package_last = 0, Vr_package_last = 0;  //上一帧下发机器人速度
    double Vr_cal = 0.0;
    double rot_factor = 0.0;

    uint8_t acc_set = 150;  //加速度限制 16ms内合成加速度最大值，单位cm/s
    uint16_t acc_r_set = 60;  //
    uint8_t DEC_FRAME = 0;
    uint8_t use_dir = 0;

    uint8_t Robot_Is_Infrared;      //红外触发
    uint8_t Robot_Is_Boot_charged;  //电容充电到60V
    // uint8_t Robot_drib;
    // uint8_t Robot_Chip_Or_Shoot;    //chip:1  shoot:0
    uint8_t Robot_Is_Shoot;
    uint8_t Robot_Is_Chip;
    uint8_t shoot_chip_flag = 0;
    // uint8_t Robot_Boot_Power = 0;
    // uint8_t Robot_Is_Report;
    uint8_t Robot_Chipped = 0, Robot_Shooted = 0;
    uint8_t Robot_Status = 0, Last_Robot_Status = 0;
    uint8_t Kick_Count = 0;
    int8_t Left_Report_Package = 0;

    uint16_t transmitted_packet = 0;
    uint32_t AD_Battery = 0, AD_Battery_Last = 217586, AD_Boot_Cap = 0;
    double Encoder_count_Motor1_avg = 0;
    double Encoder_count_Motor2_avg = 0;
    double Encoder_count_Motor3_avg = 0;
    double Encoder_count_Motor4_avg = 0;

    // uint8_t RX_Packet[25];
    void set_pid();

    void run_per_13ms(std::stop_token);
    bool get_new_pack();
    void stand();

    void period_test();

    // self test 
    void self_test();
private:
    void test_move(int Vx, int Vy, int Vr);
    void test_dribble(int d_power);
    void test_kick(int shoot_or_chip, int boot_power);
private:
    YAML::Node config_yaml;
    std::atomic_bool pid_busy = false;  // TODO: ?
    robot_comm::Robot comm_pack; 
    std::vector<int> vel_pack = {0,0,0,0};
    std::vector<int> pid_pack = {0,0,0,0,0,0,0,0};
    std::vector<int> pid_real = {0,0,0,0,0,0,0,0};
    std::vector<uint8_t> TX_Packet = {
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,  //[0-8]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,        //[9-16]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};       //[17-24]
    
    controlal control;
    // ThreadPool thpool;
    std::jthread _jthread4control,_jthread4multicast;

    int test_charge_count = 0;

    std::chrono::time_point<std::chrono::steady_clock> lasttime;

    // int infr_count = 0;
    bool valid_pack = 0;
    int chipshoot_timerdelay_flag = 0;
    
    const double Vel_k2 = config::vel_ratio;

    static constexpr double sin_angle[4] = {sin(config::car_angle_front), -sin(config::car_angle_front), -sin(config::car_angle_back), sin(config::car_angle_back)};
    static constexpr double cos_angle[4] = {-cos(config::car_angle_front), -cos(config::car_angle_front), cos(config::car_angle_back), cos(config::car_angle_back)};
    
    void pack(std::vector<uint8_t> &TX_Packet);
    int unpack(uint8_t *Packet);
    // void unpack_proto(const void* ptr, size_t size);
    void motion_planner();
    void shoot_chip();
    
    int infrare_detect();
    void infrare_toggin();

    void pid_read();
    void pid_save();

};

#endif