#ifndef ROBOT_H
#define ROBOT_H

#include "wifiz.h"
// #include "controlal.h"

#ifdef ROCKPIS_VERSION
    #include "device_ROCKS.h"
#elif defined(CM4_VERSION)
    #include "device_CM4.h"
#endif

class robotz {
public:
    robotz(int motor_num=4);
    uint8_t robot_num = 0x0f;
    
    wifi_comm wifiz;
    devicez i2c_d;
    int test_motor_num = 2;
    uint8_t motor_addr[4] = {0x28,0x29,0x30,0x31};

    // controlal bangbang;
    void testmode_on();

    // uint8_t Robot_Is_Infrared;      //红外触发
    // uint8_t Robot_Is_Boot_charged;  //电容充电到60V
    uint8_t Robot_drib;
    uint8_t Robot_Chip_Or_Shoot;    //chip:1  shoot:0
    // uint8_t Robot_Is_Shoot;
    // uint8_t Robot_Is_Chip;
    // uint8_t shoot_chip_flag = 0;
    uint8_t Robot_Boot_Power = 0;
    uint8_t Robot_Is_Report;
    // uint8_t Robot_Chipped = 0, Robot_Shooted = 0;
    // uint8_t Robot_Status = 0, Last_Robot_Status = 0;
    // uint8_t Kick_Count = 0;
    // int8_t Left_Report_Package = 0;

    int16_t Vx_package = 0, Vy_package = 0, Vr_package = 0;  //下发机器人速度
    int16_t Vx_package_last = 0, Vy_package_last = 0, Vr_package_last = 0;  //上一帧下发机器人速度
    double Vr_cal = 0.0;
    double rot_factor = 0.0;

    uint8_t acc_set = 150;  //加速度限制 16ms内合成加速度最大值，单位cm/s
    uint16_t acc_r_set = 60;  //
    uint8_t DEC_FRAME = 0;
    uint8_t use_dir = 0;

    const double Vel_k2 = 0.520573; // TODO: change to 1

    static constexpr double sin_angle[4] = {sin(58), -sin(58), -sin(45), sin(45)};
    static constexpr double cos_angle[4] = {-cos(58), -cos(58), cos(45), cos(45)};

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

    uint8_t RX_Packet[25];
    uint8_t TX_Packet[25] = {
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,  //[0-8]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,        //[9-16]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};       //[17-24]

    void pack(uint8_t *TX_Packet);
    int unpack(uint8_t *Packet);
    void motion_planner();
    void shoot_chip();

    void run();
    void regular();
    bool regular_re();
    void stand();

    int infrare_detect();
    void infrare_toggin();

    double period_test();

private:
    int vel_pack[4] = {0,0,0,0};

    int test_charge_count = 0;

    double lasttime  = 0;
    double currenttime  = 0;

    int infr_count = 0;
    bool valid_pack = 0;
};

#endif