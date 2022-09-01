#include "robotz.h"
// #include "wifiz.h"

uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;

uint8_t rxbuf[25] = {0x0};

std::mutex mutex_comm;

int main() {
    // Robot Init
    robotz zjunlict;
    zjunlict.testmode_on();

    zjunlict.robot_num = 0x0f;

    while (1)
    {   
        // Robot control
        zjunlict.run();

        // Detect receive pack
        if (Received_packet) {
            zjunlict.regular_re();
        }
        // period time test
        // zjunlict.period_test();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
