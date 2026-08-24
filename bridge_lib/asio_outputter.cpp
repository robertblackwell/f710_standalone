#include <unistd.h>
#include "asio_outputter.h"

Outputter::Outputter(boost::asio::io_context& io_context): m_io_context(io_context), m_stdin_stream(io_context){}
Outputter::~Outputter() = default;

void Outputter::send_threadsafe(std::string msg)
{
}

void Outputter::run()
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
void Outputter::start_write()
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
                std::cerr << "Log serial error: " << ec.message() << "\n";
                return;
            }
            start_write();
    });    
}