#include "nrf2401.h"
// #include "wifiz.h"
uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;
bool received_packet_flag;

uint8_t rxbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;

int main() {
    comm_2401 test;
    test.start();
    int count = 0;
    while(count++ < 10) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
