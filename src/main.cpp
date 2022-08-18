#include "main.h"
#include "robotz.h"

int vel_pack[4] = {0,0,0,0};

uint16_t Received_packet = 0;
uint8_t received_packet_flag = 0;
uint32_t Total_Missed_Package_Num = 0;

uint8_t txbuf[25] = {0x0};
uint8_t rxbuf[25] = {0x0};

int main() {
    // Udp Init
    udp_init();
    
    // Robot Init
    robotz zjunlict;
    zjunlict.motor.detect();
    // TODO: wait for robot udp init
    while ( !Received_packet )
    {
        
    }
    zjunlict.robot_num = 0x0f;
    std::mutex _mutex;

    while (1)
    {   
        // Robot control
        zjunlict.run(vel_pack);

        // Detect receive pack
        if (Received_packet) {
            // Callback of udp receive
            std::scoped_lock lock(_mutex);
            received_packet_flag = zjunlict.unpack(rxbuf);
            if (received_packet_flag) {
                // Correct package
                zjunlict.regular(vel_pack);
                Total_Missed_Package_Num = 0;
            }
            Received_packet = 0;
        }else {
            received_packet_flag = 0;
        }

        if (received_packet_flag == 0) {
            // Missing package for 1 seconds
            if (Total_Missed_Package_Num++ >= 500) {
                zjunlict.stand(vel_pack);   // Set to 0
                udp_restart();
            }
        }
        
        // period time test
        // zjunlict.period_test();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
