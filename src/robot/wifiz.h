#ifndef WIFI_H
#define WIFI_H

#include "zos/socket.h"
#include "zos/log.h"
#include <fmt/core.h>
#include "config.h"

extern uint16_t Received_packet;
extern uint8_t rxbuf[25];
extern std::string rxbuf_proto;
extern std::mutex mutex_comm;

class wifi_comm {
public:
    // std::thread receiveThread;
    wifi_comm();
    void udp_stop();
    void udp_restart();
    void udp_receiver();
    void udp_sender(std::vector<uint8_t> &txbuf);
    void udp_sender_mc();

private:
    asio::ip::udp::endpoint receiver_endpoint;
    asio::ip::udp::endpoint multicast_endpoint;

    zos::udp::socket socket_send_single;
    zos::udp::socket socket_send_multicast;
};


#endif