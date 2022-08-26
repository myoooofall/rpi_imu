#include "robotz.h"

robotz::robotz(int motor_num) : motor(motor_num) {
    // Motor Init
    // TODO: change to while
    int motors_num = motor.motors_detect();
    if ( motors_num == 0 ) {
        std::cout << "NO device detected" << std::endl;
    }else {
        std::cout << motors_num << " motor detected!" << std::endl;
    }
    // bangbang.control_init();
}

//解包，到每个轮子的速度
int robotz::unpack(uint8_t *Packet) {
    int received_packet_flag = 0;
    if ((Packet[0] & 0xf0) == 0x40) {
        if ((robot_num == (Packet[1] & 0x0f)) && (Packet[0] & 0x08)) {
            received_packet_flag = 1;
            Vx_package = (Packet[2] & 0x7f) + ((Packet[17] & 0xc0) << 1);
            Vx_package = (Packet[2] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[3] & 0x7f) + ((Packet[17] & 0x30) << 3);
            Vy_package = (Packet[3] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[4] & 0x7f) + ((Packet[17] & 0x0f) << 7);
            Vr_package = (Packet[4] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[1] >> 7;
            Robot_drib = (Packet[1] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[1] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[21] & 0x7f;
            use_dir = Packet[21] & 0x80;
        }
        else if((robot_num == (Packet[5] & 0x0f)) && (Packet[0] & 0x04)){
            received_packet_flag = 1;
            Vx_package = (Packet[6] & 0x7f) + ((Packet[18] & 0xc0) << 1);
            Vx_package = (Packet[6] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[7] & 0x7f) + ((Packet[18] & 0x30) << 3);
            Vy_package = (Packet[7] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[8] & 0x7f) + ((Packet[18] & 0x0f) << 7);
            Vr_package = (Packet[8] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[5] >> 7;
            Robot_drib = (Packet[5] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[5] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[22] & 0x7f;
            use_dir = Packet[22] & 0x80;
        }
        else if((robot_num == (Packet[9] & 0x0f)) && (Packet[0] & 0x02)){
            received_packet_flag = 1;
            Vx_package = (Packet[10] & 0x7f) + ((Packet[19] & 0xc0) << 1);
            Vx_package = (Packet[10] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[11] & 0x7f) + ((Packet[19] & 0x30) << 3);
            Vy_package = (Packet[11] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[12] & 0x7f) + ((Packet[19] & 0x0f) << 7);
            Vr_package = (Packet[12] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[9] >> 7;
            Robot_drib = (Packet[9] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[9] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[23] & 0x7f;
            use_dir = Packet[23] & 0x80;
        }
        else if((robot_num == (Packet[13] & 0x0f)) && (Packet[0] & 0x01)){
            received_packet_flag = 1;
            Vx_package = (Packet[14] & 0x7f) + ((Packet[20] & 0xc0) << 1);
            Vx_package = (Packet[14] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[15] & 0x7f) + ((Packet[20] & 0x30) << 3);
            Vy_package = (Packet[15] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[16] & 0x7f) + ((Packet[20] & 0x0f) << 7);
            Vr_package = (Packet[16] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[13] >> 7;
            Robot_drib = (Packet[13] >> 4) & 0x03;                                      
            Robot_Chip_Or_Shoot = ( Packet[13] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[24] & 0x7f;
            use_dir = Packet[24] & 0x80;
        }else 
            received_packet_flag = 0;
    }else
        received_packet_flag = 0;
    
    return received_packet_flag;
}

void robotz::motion_planner(int* vel_pack) {
    int16_t acc_x = 0;
    int16_t acc_y = 0;
    double acc_whole = 0;
    double sin_x = 0;
    double sin_y = 0;
    if(sqrt(Vx_package_last * Vx_package_last + Vy_package_last * Vy_package_last) > 325.0) {
        acc_set = 25.0;	// 4.18 test
        DEC_FRAME++;
    }else {
        DEC_FRAME = 0;
        acc_set = 20.0;
    }
    acc_x = Vx_package - Vx_package_last;
    acc_y = Vy_package - Vy_package_last; 
    acc_whole = acc_x * acc_x + acc_y * acc_y ;
    acc_whole = sqrt(acc_whole);
    sin_x = acc_x / acc_whole;
    sin_y = acc_y / acc_whole;

    if (acc_whole > acc_set) {
        acc_whole = acc_set;
        acc_x = acc_whole * sin_x;
        acc_y = acc_whole * sin_y;
        Vx_package = Vx_package_last + acc_x;
        Vy_package = Vy_package_last + acc_y; 
    }
        
    // double type
    Vr_cal = Vr_package / 160.0;
    if(!DEC_FRAME) {
        Vx_package_last = Vx_package;
        Vy_package_last = Vy_package;
    }else {
        Vx_package_last += acc_x;
        Vy_package_last += acc_y;
    }
    
    vel_pack[0] = ((sin_angle[0]) * Vx_package + (cos_angle[0]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    vel_pack[1] = ((sin_angle[1]) * Vx_package + (cos_angle[1]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    vel_pack[2] = ((sin_angle[2]) * Vx_package + (cos_angle[2]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    vel_pack[3] = ((sin_angle[3]) * Vx_package + (cos_angle[3]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
}

void robotz::stand(int* vel_pack) {
    Vy_package = 0;
    Vx_package = 0;
    Vr_package = 0;
    use_dir = 0;
    Robot_drib = 0;
                    
    std::fill_n (vel_pack, MAX_MOTOR, 0);
}

void robotz::regular(int* vel_pack) {
    motion_planner(vel_pack);
    if ((Robot_Status != Last_Robot_Status) || (Robot_Is_Infrared) || (Robot_Is_Report == 1)) {
        Left_Report_Package = 4;
        Last_Robot_Status = Robot_Status;
    }
    
    if(Kick_Count > 0)
        Kick_Count--;
    else
        Robot_Status &= 0xCF;
        
    if(Left_Report_Package > 0 || Kick_Count > 0){
        pack(TX_Packet);
        udp_sender(TX_Packet);
        transmitted_packet++;
    }

    if(Left_Report_Package > 0)   Left_Report_Package --;
}

void robotz::pack(uint8_t *TX_Packet){
    int temp_bat;
    int temp_boot;
    
    memset(TX_Packet, 0, 25);
    TX_Packet[0] = 0xff;
    TX_Packet[1] = 0x02;
    TX_Packet[2] = robot_num;
    TX_Packet[3] = Robot_Status;
    
    temp_bat = ((int32_t)AD_Battery_Last - 189600) / 50;
    
    if(temp_bat > 255){
        temp_bat = 255;
    }
    else if(temp_bat < 0){
        temp_bat = 0;
    }
    
    TX_Packet[4] = temp_bat;
    
    // Capacitor voltage: AD_Boot_Cap / 3.3 * 65536 / 10 * 1010
    temp_boot = ((int32_t)AD_Boot_Cap * 1010 / 196608);     //TODO: NEW_POWER
    
    if (temp_boot < 0){
        temp_boot = 0;
    }else if (temp_boot > 255){
        temp_boot = 255;
    }


    // TODO: define IMU
    #ifdef IMU_TEST_1
    memcpy(TX_Packet+14, IMU_RX_Buffer+4, 10);
    TX_Packet[24]= IMU_RX_Buffer[2];
    #endif
    
    TX_Packet[5] = temp_boot;

    //TX_Packet[6] = Robot_Boot_Power;
    //TX_Packet[7] = transmitted_packet;
    //uint32_t tick = HAL_GetTick();
    //TX_Packet[8] = tick >> 8 & 0xFF;
    //TX_Packet[9] = tick & 0xFF;
    //TX_Packet[10] = Left_Report_Package;
    int temp1 = Encoder_count_Motor1_avg;
    int temp2 = Encoder_count_Motor2_avg;
    int temp3 = Encoder_count_Motor3_avg;
    int temp4 = Encoder_count_Motor4_avg;
    TX_Packet[6] = (temp1 & 0xFF00)>>8;
    TX_Packet[7] = (temp1 & 0xFF);
    TX_Packet[8] = (temp2 & 0xFF00)>>8;
    TX_Packet[9] = (temp2 & 0xFF);
    TX_Packet[10] = (temp3 & 0xFF00)>>8;
    TX_Packet[11] = (temp3 & 0xFF);
    TX_Packet[12] = (temp4 & 0xFF00)>>8;
    TX_Packet[13] = (temp4 & 0xFF);
}

void robotz::run(int* vel_pack) {
    motor.motors_write(vel_pack);      // FIXME: delay period

    //infrare
    // infrare_detect();
    // if (!Robot_Is_Infrared) {
    //     std::cout << "infrare triggered: " << infr_count++ << std::endl;
    // }

    //dirbble
    
    // shoot and chip
    // TODO: Shoot and chip-need to determine time flag
    // if(chipshoot_timerdelay_flag < 1000)
    //     chipshoot_timerdelay_flag++;
    Robot_Is_Boot_charged = gpio_test.shoot_gpio(Robot_Is_Boot_charged, Robot_Boot_Power);

    // period_test();
}

double robotz::period_test() {
    double period_time;
    currenttime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()%1000000000;
    period_time = (currenttime - lasttime)/1000;
    std::cout << "period time: " << period_time << std::endl;
    lasttime = currenttime;
    return period_time;
}

void robotz::testmode_on() {
    motor.output_test();
}