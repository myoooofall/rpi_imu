#ifndef ROBOT_H
#define ROBOT_H

#ifdef OLD_VERSION
#include "nrf2401.h"
#else
#include "wifiz.h"
#endif

#include "robot_comm.pb.h"
#include "controlal.h"
// #include "controlal.h"

#ifdef ROCKPIS_VERSION
    #include "device_ROCKS.h"
#elif defined(CM4_VERSION)
    // #include "device_CM4.h"
    #include "device_pigpio.h"
#endif

class robotz {
public:
    robotz(int motor_num=4);
    uint8_t robot_num = config::robot_id;
    devicez gpio_devices;
    #ifdef OLD_VERSION
    comm_2401 comm;
    #else
    wifi_comm wifiz;
    #endif

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

    void run_per_13ms();
    bool get_new_pack();
    void stand();

    void period_test();

private:
    robot_comm::Robot comm_pack; 
    std::vector<int> vel_pack = {0,0,0,0};
    std::vector<uint8_t> TX_Packet = {
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,  //[0-8]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,        //[9-16]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};       //[17-24]
    
    controlal control;
    ThreadPool thpool;

    int test_charge_count = 0;

    std::chrono::time_point<std::chrono::steady_clock> lasttime;

    int infr_count = 0;
    bool valid_pack = 0;
    
    const double Vel_k2 = config::vel_ratio;

    static constexpr double sin_angle[4] = {sin(config::car_angle), -sin(config::car_angle), -sin(45), sin(45)};
    static constexpr double cos_angle[4] = {-cos(config::car_angle), -cos(config::car_angle), cos(45), cos(45)};
    
    void pack(std::vector<uint8_t> &TX_Packet);
    int unpack(uint8_t *Packet);
    bool unpack_proto(std::string proto_string);
    void motion_planner();
    void shoot_chip();
    
    int infrare_detect();
    void infrare_toggin();

};

#endif