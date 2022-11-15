#include "wifiz.h"

void _cb(const void* p,size_t lens) {
    std::scoped_lock lock(mutex_comm);
    std::string s(static_cast<const char*>(p),lens);
    // 更新最新数据包
    Received_packet = 1;
    std::copy(begin(s), end(s), std::begin(rxbuf));
}

wifi_comm::wifi_comm() {
    receiver_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string("127.0.0.1"), config::send_single_port);
    multicast_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(config::multicast_addr), config::send_multicast_port);

    std::thread receiveThread(&wifi_comm::udp_receiver, this);
    receiveThread.detach();
}

void wifi_comm::udp_stop() {
    zos::__io::_()->stop();
}

void wifi_comm::udp_restart() {
    zos::__io::_()->restart();
}

// Receiver
void wifi_comm::udp_receiver() {
    asio::ip::udp::endpoint listen_ep(asio::ip::address::from_string("0.0.0.0"), config::receive_port);
    zos::udp::socket socket;
    socket.bind(listen_ep,_cb);
    zos::__io::_()->run();
}

void wifi_comm::udp_sender(std::vector<uint8_t> &txbuf) {
    socket_send_single.send_to(fmt::format("send data: {}",fmt::join(txbuf, " ")), receiver_endpoint);   // TODO: protobuf
    // std::cout << "send 1 message" <<std::endl;
}

void wifi_comm::udp_sender_mc() {
    socket_send_multicast.send_to(fmt::format("muliticast test"), multicast_endpoint);
    zos::info("multicast test\n");
}