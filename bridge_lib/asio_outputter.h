#ifndef F710_OUTPUTTER_H
#define F710_OUTPUTTER_H
#include <string>
#include <queue>
#include <memory>
#include <boost/asio.hpp>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>
#include <rbl/cv_queue.h>
using namespace rbl;
class Outputter
{
public:
    Outputter(boost::asio::io_context& io_context);
    ~Outputter();
    void send_threadsafe(std::string msg);
    void run();
private:
    void start_write();
    boost::asio::posix::stream_descriptor m_stdin_stream;
    boost::asio::io_context& m_io_context;
    std::deque<IoBuffer::UPtr> m_output_queue; 
    IoBuffer::UPtr                              m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written


};


#endif //F710_OUTPUTTER_H
