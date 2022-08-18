
#include <iostream>
#include <string>
#include <functional>
#include <asio.hpp>

const short multicast_port = 30001;

class receiver
{
public:
    receiver(asio::io_context &io_context,
             const asio::ip::address &listen_address,
             const asio::ip::address &multicast_address)
        : socket_(io_context)
    {
        // Create the socket so that multiple may be bound to the same address.
        asio::ip::udp::endpoint listen_endpoint(
            listen_address, multicast_port);
        socket_.open(listen_endpoint.protocol());
        socket_.set_option(asio::ip::udp::socket::reuse_address(true));
        socket_.bind(listen_endpoint);

        // Join the multicast group.
        socket_.set_option(
            asio::ip::multicast::join_group(multicast_address));

        socket_.async_receive_from(
            asio::buffer(data_, max_length), sender_endpoint_,
            std::bind(&receiver::handle_receive_from, this,
                        std::placeholders::_1,
                        std::placeholders::_2));
    }

    void handle_receive_from(const std::error_code &error,
                             size_t bytes_recvd)
    {
        if (!error)
        {
            std::cout.write(data_, bytes_recvd);
            std::cout << std::endl;

            socket_.async_receive_from(
                asio::buffer(data_, max_length), sender_endpoint_,
                std::bind(&receiver::handle_receive_from, this,
                            std::placeholders::_1,
                            std::placeholders::_2));
        }
    }

private:
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint sender_endpoint_;
    enum
    {
        max_length = 1024
    };
    char data_[max_length];
};

class UDPSocket{};