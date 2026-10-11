#include <utility>
#include <cerrno>
#include <rbl/std_format.h>
#include <functional>
#include <memory>
#include <string>
#include <sys/select.h>
#include <chrono>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <rbl/logger.h>
#include "serial_bridge.h"
#include <f710-common/serial_settings.h>

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

bridge::asio::SerialLink::SerialLink(boost::asio::io_context& io_context): m_io_context(io_context), m_output_queue{}, m_serial_stream(io_context)
{
    m_serial_fd = -1;
    m_recv_callback = nullptr;
    m_write_buffer_uptr = nullptr;
}
bridge::asio::SerialLink::~SerialLink()
{
    close(m_serial_fd);
}
void bridge::asio::SerialLink::run(OnRecvCallback on_recv_callback)
{
    m_recv_callback = std::move(on_recv_callback);
    //
    // Step 1 Find and open the serial port
    //
    RBL_LOG_FMT("looking for serial device")
    auto port_path_list = list_serial_devices();
    if (port_path_list.size() != 1) {
        throw std::runtime_error("could not find exactly one suitable serial port path");
    }
    auto port = port_path_list[0];
    int fd = open_serial(port);
    m_serial_fd = fd;
    RBL_LOG_FMT("succeeded in openning %s ", port.c_str())
    apply_default_settings(fd);
    boost::system::error_code ec;
    const auto x = m_serial_stream.assign(m_serial_fd, ec);
    std::string y = x.to_string();
    if(ec) {
        std::cerr << "Assigning serial fd to stream failed:  " << ec.message() << "\n";
    }

    //
    // Step2 post the start function to run once the io_ctx is running
    //
    boost::asio::post(m_io_context, [this](){
        start_read();
    });
    //
    // Step 3 start the write cycle
    //
    boost::asio::post(m_io_context, [this](){
        start_writes();
    });
}
void bridge::asio::SerialLink::start_read()
{
    m_input_message_buffer_uptr = std::make_unique<rbl::IoBuffer>();
    boost::asio::async_read_until(m_serial_stream, m_read_streambuf, '\n', 
        [this](const boost::system::error_code& ec, std::size_t bytes_transfered){
            if(!ec) {
                auto bufs = m_read_streambuf.data();
                auto bufs_begin = boost::asio::buffers_begin(bufs);
                auto bufs_end = bufs_begin + bytes_transfered;
                std::string s(bufs_begin, bufs_end);
                m_read_streambuf.consume(bytes_transfered);
                auto iob = std::make_unique<rbl::IoBuffer>(s);
                m_recv_callback(std::move(iob));
            } else {
                std::cerr << "Log serial error: " << ec.message() << "\n";
            }
            start_read();
        });
}
void bridge::asio::SerialLink::start_writes()
{
    if(m_output_queue.empty()) {
        return;
    } 
    if(m_write_buffer_uptr) {
        return;
    }
    m_write_buffer_uptr = std::move(m_output_queue.front());
    m_output_queue.pop_front();
    const auto asio_buf = boost::asio::buffer(m_write_buffer_uptr->data(), m_write_buffer_uptr->size());
    boost::asio::async_write(m_serial_stream, asio_buf, 
        [this](const boost::system::error_code& ec, std::size_t bytes_transfered){
            if(!ec) {
                m_write_buffer_uptr = nullptr;
            } else {
                std::cerr << "Log serial error: " << ec.message() << "bytes_transfered " << bytes_transfered << "\n";
                return;
            }
            start_writes();
    });
}
void bridge::asio::SerialLink::send_threadsafe(rbl::IoBuffer::UPtr buffer_uptr)
{
    auto up = std::make_unique<std::string>(buffer_uptr->c_str());
    m_output_queue.push_back(std::move(buffer_uptr));
    start_writes();
}

