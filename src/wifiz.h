#ifndef WIFI_H
#define WIFI_H

#include "udp/socket.h"
#include <fmt/core.h>
#include "main.h"

extern uint16_t Received_packet;
extern uint8_t rxbuf[25];
// std::thread receiveThread;

void udp_init();
void udp_stop();
void udp_restart();
void udp_receiver();
void udp_sender(uint8_t *txbuf);

#endif