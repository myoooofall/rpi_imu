#ifndef CONFIG_H
#define CONFIG_H

#include <chrono>

#define MAX_MOTOR   4
#define TX_BUF_SIZE 25

#define DEBUG_MODE

namespace config {
    constexpr uint8_t robot_id = 0x0f;
    constexpr uint8_t motors_addr[4] = {0x28,0x29,0x30,0x31};
    constexpr uint8_t adc_addr = 0x48;
    constexpr int radio_tx_ce_pin = 22;
    constexpr int radio_tx_csn = 1; // <a>*10+<b>; spidev1.0 is 10, spidev1.1 is 11 etc..
    constexpr int radio_rx_ce_pin = 27;
    constexpr int radio_rx_csn = 0;
    constexpr double car_angle = 58;
    constexpr double vel_ratio = 0.520573;

    constexpr int receive_port = 30001;
    constexpr int send_single_port = 30002;
    constexpr int send_multicast_port = 30003;
    constexpr char multicast_addr[] = "233.233.233.233";

    constexpr int udp_freq = 1000;
    constexpr int robot_freq = 500;
}

#endif
