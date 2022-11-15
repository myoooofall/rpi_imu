#include "robotz.h"
// #include "wifiz.h"

uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;
bool received_packet_flag;

uint8_t rxbuf[25] = {0x0};

std::mutex mutex_comm;

int main() {
    // Robot Init
    robotz zjunlict;
    // zjunlict.testmode_on();
    // TODO: wait for robot udp init
    while ( !Received_packet )
    {
        
    }

    while (1)
    {
        // Detect receive pack
        received_packet_flag = 0;
        if (Received_packet) {  // Callback of udp receive
            received_packet_flag = zjunlict.regular_re();
        }
        if (received_packet_flag == 0) {
            if (Total_Missed_Package_Num++ >= 500) {    // Missing package for 1 seconds
                zjunlict.stand();   // Set to 0
            }
        }
        
        // period time test
        // zjunlict.period_test();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
