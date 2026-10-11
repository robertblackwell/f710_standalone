#ifndef H_bridge_asio_outputter_H
#define H_bridge_asio_outputter_H
#include <string>
#include <queue>
#include <memory>
#include <boost/asio.hpp>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>
#include <rbl/cv_queue.h>

namespace bridge::asio {
class Outputter
{
public:
    Outputter(boost::asio::io_context& io_context);
    ~Outputter();
    void send_threadsafe(std::string& msg);
    void run();
private:
    void start_write();
    boost::asio::posix::stream_descriptor m_stdin_stream;
    boost::asio::io_context& m_io_context;
    std::deque<rbl::IoBuffer::UPtr> m_output_queue; 
    rbl::IoBuffer::UPtr m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written
};
} // namespace

#endif 