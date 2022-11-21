#define OLD_VERSION

#include "robotz.h"
uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;
bool received_packet_flag = false;

uint8_t rxbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;

int main() {
    // comm_2401 test;
    // test.start();

    robotz zjunlict;
    // zjunlict.testmode_on();

    int count = 0;
    while(true) {
        if (zjunlict.comm.get_receive_flag()) {
            {
                std::scoped_lock lock(mutex_comm);
                // copy
                uint8_t* rxbuf_ptr = zjunlict.comm.get_rxbuf();
                std::copy(rxbuf_ptr, rxbuf_ptr+MAX_SIZE, rxbuf);
                zjunlict.comm.set_receive_flag();
            }
            // zos::log("receive package: {} {} {} {} {}\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4]);
            // zos::info("vx: {}   vy: {}\n", (rxbuf[2] & 0x7f) + ((rxbuf[17] & 0xc0) << 1), (rxbuf[3] & 0x7f) + ((rxbuf[17] & 0x30) << 3));

            received_packet_flag = zjunlict.get_new_pack();
        }

        if (received_packet_flag == 0) {
            if (Total_Missed_Package_Num++ >= 500) {    // Missing package for 1 seconds
                // zjunlict.stand();   // Set to 0
            }
        }
        // zjunlict.period_test();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
