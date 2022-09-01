#ifndef WIFI_H
#define WIFI_H

#include "socket.h"
#include <fmt/core.h>

extern uint16_t Received_packet;
extern uint8_t rxbuf[25];
extern std::mutex mutex_comm;

class wifi_comm {
public:
    // std::thread receiveThread;
    wifi_comm();
    void udp_stop();
    void udp_restart();
    void udp_receiver();
    void udp_sender(uint8_t *txbuf);

private:
};


#endif