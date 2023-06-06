#include <iostream>
#include "robotz.h"

std::atomic_bool Received_packet = 0;

uint8_t rxbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;
int main(){
    robotz zjunlict;
    while (true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}