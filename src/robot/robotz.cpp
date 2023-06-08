#include "robotz.h"
using PB = ZSS::New::Robot_Command;
robotz::robotz(int _comm_type,int motor_num)
    : gpio_devices(motor_num)
    , wifiz(std::bind(&robotz::_wifi_cb,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3))
    , control(controlal::Mode::RealTime)
    , _cmd_subsciber("cmd",std::bind(&robotz::_cmd_cb,this,std::placeholders::_1))
    , _cmd_publisher("cmd"){
    // Motor Init
    // TODO: change to while
    int motors_num = gpio_devices.motors_detect();
    if ( motors_num == 0 ) {
        zos::log("NO motor detected\n");
    }else {
        zos::log("{} motor detected!\n", motors_num);
    }
    pid_read();
    set_pid();
    // bangbang.control_init();
    // #ifdef OLD_VERSION
    // comm.start();
    // #endif
    
    // thpool.enqueue(&robotz::run_per_13ms, this);
    if(_comm_type == COMM_TYPE_24L01){
        _jthread4control = std::jthread(std::bind(&robotz::run_per_13ms,this,std::placeholders::_1));
    }else if(_comm_type == COMM_TYPE_WIFI){
        _jthread4control = std::jthread(std::bind(&robotz::_control_thread,this,std::placeholders::_1));
        _jthread4multicast = std::jthread(std::bind(&robotz::_multicast_info_thread,this,std::placeholders::_1));
        _jthread4sendback = std::jthread(std::bind(&robotz::_send_status_thread,this,std::placeholders::_1));
    }
}

//解包，到每个轮子的速度
int robotz::unpack(uint8_t *Packet) {
    #ifdef OLD_VERSION
    std::scoped_lock lock(nrf2401.mutex_comm_2401);
    #else
    std::scoped_lock lock(mutex_comm);
    #endif
    // if ((Packet[0] & 0xf0) == 0x40)

    if ((Packet[0] & 0xf0) == 0x40) {
        if ((robot_num == (Packet[1] & 0x0f)) && (Packet[0] & 0x08)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[2] & 0x7f) + ((Packet[17] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[2] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vy_target = (Packet[3] & 0x7f) + ((Packet[17] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[3] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vr_target = (Packet[4] & 0x7f) + ((Packet[17] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[4] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.need_report = Packet[1] >> 7;
            robot_cmd.drib_power = (Packet[1] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = Packet[21] & 0x7f;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[1] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[21] & 0x80;
        }else if ((robot_num == (Packet[5] & 0x0f)) && (Packet[0] & 0x04)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[6] & 0x7f) + ((Packet[18] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[6] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vy_target = (Packet[7] & 0x7f) + ((Packet[18] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[7] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vr_target = (Packet[8] & 0x7f) + ((Packet[18] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[8] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.need_report = Packet[5] >> 7;
            robot_cmd.drib_power = (Packet[5] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = Packet[22] & 0x7f;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[5] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[22] & 0x80;
        }else if ((robot_num == (Packet[9] & 0x0f)) && (Packet[0] & 0x02)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[10] & 0x7f) + ((Packet[19] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[10] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vy_target = (Packet[11] & 0x7f) + ((Packet[19] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[11] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vr_target = (Packet[12] & 0x7f) + ((Packet[19] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[12] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.need_report = Packet[9] >> 7;
            robot_cmd.drib_power = (Packet[9] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = Packet[23] & 0x7f;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[9] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[23] & 0x80;
        }else if ((robot_num == (Packet[13] & 0x0f)) && (Packet[0] & 0x01)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[14] & 0x7f) + ((Packet[20] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[14] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vy_target = (Packet[15] & 0x7f) + ((Packet[20] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[15] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vr_target = (Packet[16] & 0x7f) + ((Packet[20] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[16] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.need_report = Packet[13] >> 7;
            robot_cmd.drib_power = (Packet[13] >> 4) & 0x03;                                      
            robot_cmd.kick_discharge_time = Packet[24] & 0x7f;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[13] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[24] & 0x80;
        }else 
            valid_pack = 0;
        zos::info("vx: {}   vy: {}   vr: {}\n", robot_cmd.vx_target, robot_cmd.vy_target, robot_cmd.vr_target);
    }else if (Packet[0] == 0xab) {
        // pid_pack[0] = 0xf5;
        // pid_pack[1] = Packet[1];
        // pid_pack[2] = Packet[2];
        if(Packet[1] >> 7 == 1) {
            pid_pack[0] = Packet[2];
            pid_pack[1] = Packet[3];
        }
        if(Packet[5] >> 7 == 1) {
            pid_pack[2] = Packet[6];
            pid_pack[3] = Packet[7];
        }
        if(Packet[9] >> 7 == 1) {
            pid_pack[4] = Packet[10];
            pid_pack[5] = Packet[11];
        }
        if(Packet[13] >> 7 == 1) {
            pid_pack[6] = Packet[14];
            pid_pack[7] = Packet[15];
        }
        
        robot_cmd.need_report = 1;
        valid_pack = 1;
        zos::info("p: {}   i: {}\n", pid_pack[0], pid_pack[1]);
        set_pid();
    }else {
        valid_pack = 0;
        zos::error("wrong pack: {} {} {}\n", Packet[0], Packet[1], Packet[2]);
    }
    
    return valid_pack;
}

// bool robotz::unpack_proto(const void* ptr, size_t size) {
//     bool valid = false;
//     // valid = comm_pack.ParseFromString(proto_string);
//     // // zos::info("parsing protobuf\n");
//     // if (valid) {
//     //     zos::info("Proto test robot_cmd.vx_target: {}\n", comm_pack.robot_cmd.vx_target());
//     //     zos::info("Proto test robot_cmd.vy_target: {}\n", comm_pack.robot_cmd.vy_target());
//     // }
//     return valid;
// }
void robotz::_wifi_cb(const asio::ip::udp::endpoint& ep,const void* ptr,size_t size){
    wifiz.set_master_ip(ep.address().to_string());
    {
        std::scoped_lock lock{_cmd_data_mutex};
        zos::warning("befor cmd_store\n");
        _cmd_data.resize(size);
        _cmd_data.store(ptr,size);
        zos::warning("after cmd_store\n");
    }

    _cmd_publisher.publish();
}
void robotz::_cmd_cb(const zos::Data& data){
    std::scoped_lock lock{_cmd_data_mutex};
    zos::warning("before parsefromarray\n");
    auto res = pb_cmd.ParseFromArray(_cmd_data.data(),_cmd_data.size());
    zos::warning("after parsefromarray\n");
    if(res){
        auto& _p = pb_cmd;
        auto& _c = robot_cmd;
        robot_status.time_since_last_pack = 0;
        zos::log("{}\n",_p.DebugString());
        // TODO update
        std::scoped_lock lock{_robot_cmd_mutex};
        // _c.id = _p.robot_id(); // maybe need to check if id correct
        _c.kick_en = (_p.kick_mode() != PB::NONE);
        _c.kick_mode = (_p.kick_mode() == PB::CHIP);
        _c.desire_power = _p.desire_power();
        _c.kick_discharge_time = _p.kick_discharge_time();
        _c.drib_power = _p.dribble_spin();
        _c.cmd_type = int(_p.cmd_type());
        switch(_p.cmd_type()){
            case PB::CMD_WHEEL:{
                _c.motor_vel_target[0] = _p.cmd_wheel().wheel1();
                _c.motor_vel_target[1] = _p.cmd_wheel().wheel2();
                _c.motor_vel_target[2] = _p.cmd_wheel().wheel3();
                _c.motor_vel_target[3] = _p.cmd_wheel().wheel4();
                break;
            }
            case PB::CMD_VEL:{
                _c.vx_target = _p.cmd_vel().velocity_x();
                _c.vy_target = _p.cmd_vel().velocity_y();
                _c.use_dir = _p.cmd_vel().use_imu();
                _c.vr_target = _c.use_dir ? _p.cmd_vel().imu_theta() : _p.cmd_vel().velocity_r();
                break;
            }
            default:{
                zos::error("current cmd_type not supported:{}\n",_p.cmd_type());
                break;
            }
        }
    }
}
void robotz::_update_status(const double dt){
    std::vector<int> nano_pack = gpio_devices.read_nano_uart();

    {
        std::scoped_lock lock{_robot_status_mutex};
        auto& _s = robot_status;
        if(nano_pack[0]){
            if(_s.robot_is_infrared > 0){
                _s.robot_is_infrared += dt;
            }else{
                _s.robot_is_infrared = 1;
            }
        }else{
            _s.robot_is_infrared = -1;
        }
        _s.cap_vol = nano_pack[1];
        _s.bat_vol = nano_pack[2]/10.0;
        _s.motor_encoder = gpio_devices.get_encoder_array();
        // kick status will be reset to 0 in control thread (no closed loop check)
        _s.last_kick_time = std::min(_s.last_kick_time+dt,10000.0);
        _s.robot_is_shooted = std::min(_s.robot_is_shooted+dt,10000.0);
        _s.robot_is_chipped = std::min(_s.robot_is_chipped+dt,10000.0);
    }
}
void robotz::_send_status_thread(std::stop_token _stop_token){
    zos::Rate robot_rate(config::sendback_freq);
    zos::Data data;
    ZSS::New::Robot_Status pb_status;
    while(true){
        if(_stop_token.stop_requested()) break;
        _update_status(1000.0/config::sendback_freq);
        if(wifiz.get_master_ip() != ""){
            break;
        }
        robot_rate.sleep();
    }
    while(true){
        if(_stop_token.stop_requested()) break;
        // TODO
        _update_status(1000.0/config::sendback_freq); // ms
        {
            std::scoped_lock lock{_robot_status_mutex};
            auto& _s = robot_status;
            pb_status.set_robot_id(_s.id);
            pb_status.set_infrared(_s.robot_is_infrared);
            pb_status.set_flat_kick(_s.robot_is_shooted);
            pb_status.set_chip_kick(_s.robot_is_chipped);
            pb_status.set_battery(_s.bat_vol);
            pb_status.set_capacitance(_s.cap_vol);
            pb_status.add_wheel_encoder(_s.motor_encoder[0]);
            pb_status.add_wheel_encoder(_s.motor_encoder[1]);
            pb_status.add_wheel_encoder(_s.motor_encoder[2]);
            pb_status.add_wheel_encoder(_s.motor_encoder[3]);
        }
        auto size = pb_status.ByteSizeLong();
        data.resize(size);
        pb_status.SerializeToArray(data.ptr(),size);
        wifiz.udp_sender(data.data(),size);
        pb_status.Clear();
        robot_rate.sleep();
    }
}
void robotz::_multicast_info_thread(std::stop_token _stop_token){
    zos::Rate robot_rate(config::multicast_freq);
    zos::Data data;
    ZSS::New::Multicast_Status mc_status;
    while(true){
        robot_status.time_since_last_pack += 1000.0 / config::multicast_freq; // ms
        if(_stop_token.stop_requested()) break;
        // TODO
        mc_status.set_ip(wifiz.get_ip());
        mc_status.set_uuid("0000-0000-0000-0000");
        mc_status.set_team(ZSS::New::Multicast_Status_Team_BLUE);
        mc_status.set_robot_id(6); // ROBOT_ID change 
        mc_status.set_battery(-1.0);
        mc_status.set_capacitance(-1.0);
        auto size = mc_status.ByteSizeLong();
        data.resize(size);
        mc_status.SerializeToArray(data.ptr(),size);
        wifiz.udp_sender_mc(data.data(),size);
        mc_status.Clear();
        robot_rate.sleep();
    }
}
void robotz::_control_thread(std::stop_token _stop_token){
    zos::Rate robot_rate(config::control_cmd_freq);
    while(true){
        Robot_CMD _c;
        {
            std::scoped_lock lock{_robot_cmd_mutex};
            _c = robot_cmd;
        }
        {
            switch(_c.cmd_type){
                case CMD_TYPE_NONE:{
                    break;
                }
                case CMD_TYPE_WHEEL:{
                    break;
                }
                case CMD_TYPE_VEL:{
                    motion_planner();
                    break;
                }
                default:{
                    zos::error("_control_thread check cmd_type not supported\n");
                    break;
                }
            }
        }
        zos::info("motor set : {},{},{},{}\n",vel_pack[0],vel_pack[1],vel_pack[2],vel_pack[3]);
        gpio_devices.motors_write(vel_pack);
        
        robot_rate.sleep();
    }
}

void robotz::motion_planner() {
    // int16_t acc_x = 0;
    // int16_t acc_y = 0;
    // double acc_whole = 0;
    // double sin_x = 0;
    // double sin_y = 0;
    // if (sqrt(robot_cmd.vx_target_last * robot_cmd.vx_target_last + robot_cmd.vy_target_last * robot_cmd.vy_target_last) > 325.0) {
    //     robot_cmd.acc_set = 25.0;	// 4.18 test
    //     robot_cmd.DEC_FRAME++;
    // }else {
    //     robot_cmd.DEC_FRAME = 0;
    //     robot_cmd.acc_set = 20.0;
    // }
    // acc_x = robot_cmd.vx_target - robot_cmd.vx_target_last;
    // acc_y = robot_cmd.vy_target - robot_cmd.vy_target_last; 
    // acc_whole = acc_x * acc_x + acc_y * acc_y ;
    // acc_whole = sqrt(acc_whole);
    // sin_x = acc_x / acc_whole;
    // sin_y = acc_y / acc_whole;

    // if (acc_whole > robot_cmd.acc_set) {
    //     acc_whole = robot_cmd.acc_set;
    //     acc_x = acc_whole * sin_x;
    //     acc_y = acc_whole * sin_y;
    //     robot_cmd.vx_target = robot_cmd.vx_target_last + acc_x;
    //     robot_cmd.vy_target = robot_cmd.vy_target_last + acc_y; 
    // }
        
    // // double type
    // if(!robot_cmd.DEC_FRAME) {
    //     robot_cmd.vx_target_last = robot_cmd.vx_target;
    //     robot_cmd.vy_target_last = robot_cmd.vy_target;
    // }else {
    //     robot_cmd.vx_target_last += acc_x;
    //     robot_cmd.vy_target_last += acc_y;
    // }

    for(int i=0; i < 4; i++) {
        vel_pack[i] = ((sin_angle[i]) * robot_cmd.vx_target + (cos_angle[i]) * robot_cmd.vy_target - 8.2 * robot_cmd.vr_target / 160.0) * Vel_k2*100;
        // vel_pack[i] = (vel_pack[i] >  4096) ?  4096 : vel_pack[i];
        // vel_pack[i] = (vel_pack[i] < -4095) ? -4095 : vel_pack[i];
    }

    // zos::info("vel_pack: {} {} {} {}\n", vel_pack[0], vel_pack[1], vel_pack[2], vel_pack[3]);    
}

void robotz::stand() {
    robot_cmd.cmd_type = CMD_TYPE_VEL;
    robot_cmd.vy_target = 0;
    robot_cmd.vx_target = 0;
    robot_cmd.vr_target = 0;
    robot_cmd.use_dir = 0;
    robot_cmd.drib_power = 0;
                    
    std::fill_n(begin(vel_pack), MAX_MOTOR, 0);
    #ifndef OLD_VERSION
    wifiz.udp_restart();
    #endif
}

// void robotz::regular() {
//     motion_planner();
//     if ((Robot_Status != Last_Robot_Status) || (robot_status.robot_is_infrared) || (robot_cmd.need_report == 1)) {
//         Left_Report_Package = 4;
//         Last_Robot_Status = Robot_Status;
//     }
    
//     if(Kick_Count > 0)
//         Kick_Count--;
//     else
//         Robot_Status &= 0xCF;
        
//     if(Left_Report_Package > 0 || Kick_Count > 0){
//         pack(TX_Packet);
//         wifiz.udp_sender(TX_Packet);
//         robot_status.transmitted_pack_count++;
//     }

//     if(Left_Report_Package > 0)   Left_Report_Package --;
// }

void robotz::pack(std::vector<uint8_t> &TX_Packet) {
    int temp_bat;
    int temp_boot;
    
    std::fill_n(begin(TX_Packet), TX_BUF_SIZE, 0);
    TX_Packet[0] = 0xff;
    TX_Packet[1] = 0x02;
    TX_Packet[2] = robot_num;
    TX_Packet[3] = Robot_Status;
    
    temp_bat = ((int32_t)robot_status.bat_vol - 189600) / 50;    // TODO: bat power
    
    if(temp_bat > 255) {
        temp_bat = 255;
    }else if (temp_bat < 0) {
        temp_bat = 0;
    }
    
    TX_Packet[4] = temp_bat;
    
    // Capacitor voltage: robot_status.cap_vol / 3.3 * 65536 / 10 * 1010
    temp_boot = ((int32_t)robot_status.cap_vol * 1010 / 196608);     //TODO: NEW_POWER
    
    if (temp_boot < 0) {
        temp_boot = 0;
    }else if (temp_boot > 255) {
        temp_boot = 255;
    }


    // TODO: define IMU
    #ifdef IMU_TEST_1
    memcpy(TX_Packet+14, IMU_RX_Buffer+4, 10);
    TX_Packet[24]= IMU_RX_Buffer[2];
    #endif
    
    TX_Packet[5] = temp_boot;

    //TX_Packet[6] = robot_cmd.kick_discharge_time;
    //TX_Packet[7] = robot_status.transmitted_pack_count;
    //uint32_t tick = HAL_GetTick();
    //TX_Packet[8] = tick >> 8 & 0xFF;
    //TX_Packet[9] = tick & 0xFF;
    //TX_Packet[10] = Left_Report_Package;
    // int temp1 = Encoder_count_Motor1_avg;
    // int temp2 = Encoder_count_Motor2_avg;
    // int temp3 = Encoder_count_Motor3_avg;
    // int temp4 = Encoder_count_Motor4_avg;

    // encoder
    robot_status.motor_encoder = gpio_devices.get_encoder_array();
    TX_Packet[6] = ((int)(robot_status.motor_encoder[0]) & 0xFF00)>>8;
    TX_Packet[7] = ((int)(robot_status.motor_encoder[0]) & 0xFF);
    TX_Packet[8] = ((int)(robot_status.motor_encoder[1]) & 0xFF00)>>8;
    TX_Packet[9] = ((int)(robot_status.motor_encoder[1]) & 0xFF);
    TX_Packet[10] = ((int)(robot_status.motor_encoder[2]) & 0xFF00)>>8;
    TX_Packet[11] = ((int)(robot_status.motor_encoder[2]) & 0xFF);
    TX_Packet[12] = ((int)(robot_status.motor_encoder[3]) & 0xFF00)>>8;
    TX_Packet[13] = ((int)(robot_status.motor_encoder[3]) & 0xFF);

    // pid real
    TX_Packet[14] = pid_real[0];
    TX_Packet[15] = pid_real[1];
    TX_Packet[16] = pid_real[2];
    TX_Packet[17] = pid_real[3];
    TX_Packet[18] = pid_real[4];
    TX_Packet[19] = pid_real[5];
    TX_Packet[20] = pid_real[6];
    TX_Packet[21] = pid_real[7];
    // zos::log("tx package: {}\n", TX_Packet[0]);
}

void robotz::run_per_13ms(std::stop_token _stop_token) {
    zos::Rate robot_rate(config::robot_freq);
    while (true)
    {   
        if(pid_busy){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if(_stop_token.stop_requested()) break;

        gpio_devices.motors_write(vel_pack);
        // gpio_devices.charge_switch();

        //infrare
        std::vector<int> nano_pack = gpio_devices.read_nano_uart();
        zos::status("infrare: {},   cap_vol: {:3}V,   bat_vol: {:.1f}V\n", nano_pack[0], nano_pack[1], nano_pack[2]/10.0);

        if (nano_pack[0]) {
            // zos::status("infrare triggered\n"); 
            robot_status.robot_is_infrared = 1;
            Robot_Status = Robot_Status | (1 << 6);
        }else {
            robot_status.robot_is_infrared = 0;
            Robot_Status = Robot_Status & 0x30;
        }

        if(chipshoot_timerdelay_flag < 127)
            chipshoot_timerdelay_flag++;

        // dirbble
        // TODO: power -1~1
        gpio_devices.dribbler(robot_cmd.drib_power);
        // gpio_devices.dribbler(robot_cmd.drib_power | 0x0c);
        
        // shoot and chip
        // TODO: Shoot and chip-need to determine time flag
        // if(chipshoot_timerdelay_flag < 1000)
        //     chipshoot_timerdelay_flag++;
        robot_status.cap_vol = nano_pack[1];
        robot_status.bat_vol = nano_pack[2]/10.0;

        robot_rate.sleep();
        // period_test();
    }
}

bool robotz::get_new_pack() {
    if (unpack(rxbuf)) {
        // Correct package
        motion_planner();

        if (robot_cmd.kick_discharge_time && robot_status.robot_is_infrared && chipshoot_timerdelay_flag>80) {
            gpio_devices.shoot_chip(robot_cmd.kick_mode, robot_cmd.kick_discharge_time);
            // TODO: robot_status.robot_is_shooted/chipped
            chipshoot_timerdelay_flag = 0;
        }

        if ((Robot_Status != Last_Robot_Status) || (robot_status.robot_is_infrared) || (robot_cmd.need_report == 1)) {
            Left_Report_Package = 6;
            Last_Robot_Status = Robot_Status;
        }
        // zos::status("pack infrare before kick: {}\n", (Robot_Status >> 6));

        if(Kick_Count > 0)
            Kick_Count--;
        else
            Robot_Status &= 0xCF;

        if(Left_Report_Package > 0 || Kick_Count > 0) {
            pack(TX_Packet);
            zos::log("pack infrare: {:x}\n", (TX_Packet[3]));
            #ifdef OLD_VERSION
            // zos::info("ready to send\n");
            nrf2401.send(std::data(TX_Packet));
            #else
            wifiz.udp_sender(TX_Packet.data(),TX_Packet.size());
            #endif
            robot_status.transmitted_pack_count++;
        }

        if(Left_Report_Package > 0)   Left_Report_Package --;
    }

    Received_packet = false;
    return valid_pack;
}

void robotz::period_test() {
    auto currenttime = std::chrono::steady_clock::now();
    auto step = (currenttime - lasttime);
    zos::info("period time: {}\n", step.count()/1000);
    lasttime = currenttime;
}

void robotz::testmode_on() {
    gpio_devices.output_test();
}

void robotz::set_pid() {
    stand();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pid_busy = true;
    gpio_devices.motors_write_pid(pid_pack);
    pid_real = gpio_devices.get_pid();
    pid_save();
    std::fill_n(begin(pid_pack), 8, 0);
    pid_busy = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void robotz::pid_read() {
    try {
        config_yaml = YAML::LoadFile("config.yaml");
        // loop over the positions Rectangle and print them:
        for(auto motor : config_yaml["motor_pid"]) {
            int motor_id = motor["id"].as<int>();
            zos::log("motor{}  p: {}  i: {}\n", motor_id, motor["p"].as<int>(), motor["i"].as<int>());
            pid_pack[motor_id*2] = motor["p"].as<int>();
            pid_pack[motor_id*2+1] = motor["i"].as<int>();
        }

    } catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
    } catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
    }
}

void robotz::pid_save() {
    for(auto motor : config_yaml["motor_pid"]) {
        int motor_id = motor["id"].as<int>();
        if (pid_real[motor_id*2] > 0) {
            motor["p"] = pid_real[motor_id*2];
        }
        if (pid_real[motor_id*2+1] > 0) {
            motor["i"] = pid_real[motor_id*2+1];
        }
    }
    std::ofstream fout("config.yaml");
    fout << config_yaml; // dump it back into the file
    fout.close();
    zos::log("pid update\n");
    // pid_read();
}

void robotz::self_test() {
    test_move(20, 0, 0);
    test_move(0, 20, 0);
    test_move(0, 0, 60);
    
    test_dribble(1);
    test_dribble(3);

    // test_kick(0, 40);
    // test_kick(1, 40);
}

void robotz::test_move(int Vx, int Vy, int Vr) {
    robot_cmd.vx_target = Vx;
    robot_cmd.vy_target = Vy;
    robot_cmd.vr_target = Vr;
    motion_planner();
    zos::log("move x:{} y:{} z:{}\n", Vx, Vy, Vr);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_cmd.vx_target = 0;
    robot_cmd.vy_target = 0;
    robot_cmd.vr_target = 0;
    motion_planner();
}
void robotz::test_dribble(int d_power) {
    robot_cmd.drib_power = d_power;
    zos::log("dribble {}\n", d_power);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_cmd.drib_power = 0;
}
void robotz::test_kick(int shoot_or_chip, int boot_power) {
    robot_cmd.kick_mode = shoot_or_chip;
    robot_cmd.kick_discharge_time = boot_power;
    zos::log("shoot/chip:{}  power:{}\n", shoot_or_chip, boot_power);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot_cmd.kick_discharge_time = 0;
}