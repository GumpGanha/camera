#include <NDIClient.h>
#include"Json_NDIData.h"
using namespace boost::asio::ip;

#include <boost/array.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/thread.hpp>

#include <iostream>
using std::cout;
using std::endl;
//#pragma execution_character_set("utf-8")

NDIClient::NDIClient()
    :m_it(m_longbuffer.begin())
    , m_bytesReceive(0)
    , m_connectFlag(false) {
    msp_socket = boost::make_shared<boost::asio::ip::tcp::socket>(m_io);
}

NDIClient::~NDIClient() {
    close();
}

bool NDIClient::start_port(string ip_, int port_) {
    if (m_connectFlag) {
        //关闭之前的socket
        msp_socket->close();
    }

    tcp::endpoint endpoint(address::from_string(ip_), port_);
    msp_socket->async_connect(endpoint, bind(&NDIClient::connect_handler, this, boost::asio::placeholders::error, msp_socket));

    return true;
}

void NDIClient::io_run_thread(void) {
    m_iosThread = boost::make_shared<boost::thread>(boost::bind(&boost::asio::io_service::run, &m_io));
}

void NDIClient::connect_handler(const boost::system::error_code& ec_, boost::shared_ptr<boost::asio::ip::tcp::socket> socket_) {
    if (ec_) {
        m_connectFlag = false;
        cout << "connect error!" << endl;
        cout << ec_.message() << endl;
    }
    else {
        m_connectFlag = true;
        cout << "Connected!" << endl;
        socket_->async_read_some(boost::asio::buffer(m_buffer), bind(&NDIClient::read_handler, this, socket_, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void NDIClient::read_handler(boost::shared_ptr<boost::asio::ip::tcp::socket> socket_, const boost::system::error_code& ec, size_t bytes_transferred) {
    if (ec) {
        cout << ec.message() << endl;
        return;
    }
    cout << "read data" << endl;

    socket_->async_read_some(boost::asio::buffer(m_buffer), bind(&NDIClient::read_handler, this, socket_, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void NDIClient::write_handler(boost::shared_ptr<boost::asio::ip::tcp::socket> socket_, const boost::system::error_code& ec) {
    if (ec) {
        cout << "send fail" << ec.message() << endl;
        return;
    }
}

void NDIClient::write_data(NDIDataStruct& data_) {
    if (m_connectFlag) {
        cout << "write data" << endl;
        //数据转换为字符串
        Json_NDIData  Js;
        //string send = Js.create_json(data_);
        //send = Js.add_check(send);
        //数据发送
        //msp_socket->async_write_some(boost::asio::buffer(*send), boost::bind(&NDIClient::write_handler, this, msp_socket, boost::asio::placeholders::error));

        string data = Js.create_json(data_);
        string check_data = Js.add_check(data);
        boost::shared_ptr<std::string> send = boost::make_shared<std::string>(check_data);
        // 使用智能指针保证字符串生命周期
        msp_socket->async_write_some(boost::asio::buffer(*send),
            [this, send](const boost::system::error_code& ec, std::size_t /*length*/) {
                this->write_handler(msp_socket, ec);
            });
    }
    else {
        cout << "connect error, dont't send" << endl;
    }
}

bool NDIClient::get_connecteFlag(void) {
    return m_connectFlag;
}

bool NDIClient::close(void) {
    if (msp_socket && msp_socket->is_open()) {
        m_io.stop();
        msp_socket->close();
        m_iosThread->join();
    }
    m_connectFlag = false;
    return true;
}