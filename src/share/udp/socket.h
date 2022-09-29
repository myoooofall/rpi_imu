#ifndef __ZOS_UDP_SOCKET_H__
#define __ZOS_UDP_SOCKET_H__
#define FMT_HEADER_ONLY

#include <string>
#include <array>
#include <functional>
#include <asio.hpp>
#include "singleton.hpp"
namespace zos{
using __io = Singleton<asio::io_context>;
namespace udp{
using __callback_type = std::function<void(const void*,size_t)>;
class socket{
public:
    socket():_socket(__io::GetInstance(),asio::ip::udp::v4()){}
    socket(const asio::ip::udp::endpoint& ep,const __callback_type& f = {}):_listen_ep(ep),_socket(__io::GetInstance(),ep.protocol()){
        if(f){
            _callback = std::bind(f,std::placeholders::_1,std::placeholders::_2);
        }
        _socket.set_option(asio::ip::udp::socket::reuse_address(true));
        _socket.bind(_listen_ep);
        _socket.async_receive_from(asio::buffer(_data,MAX_LENGTH),_received_ep
            , std::bind(&socket::handle_receive_from, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
    socket(const char* multicast_address,const asio::ip::udp::endpoint& ep,const __callback_type& f = {}):socket(ep,f){
        _socket.set_option(asio::ip::multicast::join_group(asio::ip::address::from_string(multicast_address)));
    }
    ~socket() = default;
    void send_to(const std::string& str,const asio::ip::udp::endpoint& endpoint){
        _socket.send_to(asio::buffer(str.c_str(),str.size()),endpoint);
    }
private:
    void handle_receive_from(const std::error_code &error, size_t bytes_recvd){
        if (!error){
            if(_callback) std::invoke(_callback,_data.data(),bytes_recvd);
            _socket.async_receive_from(asio::buffer(_data,MAX_LENGTH),_received_ep
                , std::bind(&socket::handle_receive_from, this, std::placeholders::_1, std::placeholders::_2)
            );
        }
    }
private:
    asio::ip::udp::endpoint _listen_ep,_received_ep;
    asio::ip::udp::socket _socket;
    enum{
        MAX_LENGTH = 512
    };
    std::array<char,MAX_LENGTH> _data;
    __callback_type _callback = {};
};
} // namespace zos::udp
} // namespace zos
#endif // __ZOS_UDP_SOCKET_H__