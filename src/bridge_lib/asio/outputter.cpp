#include <unistd.h>
#include "outputter.h"

bridge::asio::Outputter::Outputter(boost::asio::io_context& io_context): m_stdin_stream(io_context), m_io_context(io_context)
{
}
bridge::asio::Outputter::~Outputter() = default;

void bridge::asio::Outputter::send_threadsafe(std::string& msg)
{
    auto buffer_uptr = std::make_unique<rbl::IoBuffer>(msg);
    auto up = std::make_unique<std::string>(buffer_uptr->c_str());
    m_output_queue.push_back(std::move(buffer_uptr));
    start_write();
}

void bridge::asio::Outputter::run()
{
    boost::system::error_code ec;
    m_stdin_stream.assign(STDIN_FILENO, ec);
    if(ec) {
        std::cerr << "Assigning serial fd to stream failed:  " << ec.message() << "\n";
    }
    boost::asio::post(m_io_context, [this](){
        start_write();
    });
}
void bridge::asio::Outputter::start_write()
{
    if(m_output_queue.empty()) {
        return;
    } 
    if(m_write_buffer_uptr) {
        return;
    }
    m_write_buffer_uptr = std::move(m_output_queue.front());
    m_output_queue.pop_front();
    auto asio_buf = boost::asio::buffer(m_write_buffer_uptr->data(), m_write_buffer_uptr->size());
    boost::asio::async_write(m_stdin_stream, asio_buf, 
        [this](const boost::system::error_code& ec, std::size_t bytes_transfered){
            if(!ec) {
                m_write_buffer_uptr = nullptr;
            } else {
                std::cerr << "Log serial error: " << ec.message() << " bytes_transfered: " << bytes_transfered << "\n";
                return;
            }
            start_write();
    });    
}