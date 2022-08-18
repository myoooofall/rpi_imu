#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
// #include "controlal.h"

#ifdef ROCKPIS_VERSION
    #include "i2c_sntest.h"
    #include "other_gpio.h"
#elif defined(CM4_VERSION)
    #include "i2c_sntestCM4.h"
    #include "other_gpioCM4.h"
#endif
class robotz {
public:
    robotz(int motor_num=4);
    uint8_t robot_num = 0x0f;
    
    i2c_device motor;
    int test_motor_num = 2;
    uint8_t motor_addr[4] = {0x28,0x29,0x30,0x31};

    other_gpio gpio_test;
    // controlal bangbang;

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

    double sin_angle[4] = {0.7071067811865476, -0.7071067811865476,
                        -0.7071067811865476,
                        0.7071067811865476};  // double sin_angle[4] = {sin(58),
                                                // -sin(58), -sin(45), sin(45)};
    double cos_angle[4] = {-0.7071067811865476, -0.7071067811865476,
                        0.7071067811865476,
                        0.7071067811865476};  // double cos_angle[4] = {-cos(58),
                                                // -cos(58), cos(45), cos(45)};

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
    void motion_planner(int* vel_pack);
    void shoot_chip();

    void run(int* vel_pack);
    void regular(int* vel_pack);
    void stand(int* vel_pack);

    int infrare_detect();
    void infrare_toggin();

    double period_test();

private:
    std::mutex mutex_robot;
    int test_charge_count = 0;

    double lasttime  = 0;
    double currenttime  = 0;

    int infr_count = 0;
};

#endif