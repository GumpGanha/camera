#pragma once

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include<boost/thread.hpp>

#include <string>
using std::string;

using boost::asio::io_service;
//using boost::asio::ip::tcp::socket;
#include <NDI_DataStruct.h>

#define  REICEIVE_BUFFER_SIZE  192    //186除去头和尾

const int BUFFER_SIZE = 1024;

const string ip = "192.168.163.1";//要连接的服务器的ip地址
const int  port = 16666;

class NDIClient {
public:
    NDIClient();
    ~NDIClient();

    bool start_port(string ip_ = ip, int port_ = port);

    void io_run_thread(void);

    void connect_handler(const boost::system::error_code& ec_, boost::shared_ptr<boost::asio::ip::tcp::socket> socket_);

    void read_handler(boost::shared_ptr<boost::asio::ip::tcp::socket> socket_, const boost::system::error_code& ec, size_t bytes_transferred);
    void write_handler(boost::shared_ptr<boost::asio::ip::tcp::socket> socket_, const boost::system::error_code& ec);

    void write_data(NDIDataStruct& data_);

    ////获取当前连接是否成功的标志
    bool  get_connecteFlag(void);
    //close
    bool close(void);
private:

    io_service							                    m_io;
    boost::shared_ptr<boost::asio::ip::tcp::socket>			msp_socket;
    boost::shared_ptr<boost::thread>                        m_iosThread;
    boost::array<char, REICEIVE_BUFFER_SIZE>                m_receive_buf;
    std::array<char, 300>									m_buffer;
    std::array<char, BUFFER_SIZE>							m_longbuffer;
    std::array< char, BUFFER_SIZE>::iterator				m_it;
    std::size_t												m_bytesReceive;
    bool                                                    m_connectFlag;

public:
};

typedef boost::shared_ptr<NDIClient> iNDIClient_spointer;
