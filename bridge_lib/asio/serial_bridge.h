#ifndef H_port_h
#define H_port_h
#include <utility>
#include <memory>
#include <variant>
#include <queue>
#include <boost/asio.hpp>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>


namespace bridge::asio {
    class SerialLink {
    public:
        typedef std::function<void(rbl::IoBuffer::UPtr)>  OnRecvCallback;
        typedef std::unique_ptr<SerialLink>          UPtr; 
        /**
         * The constructor only initializes a few member variables.
         * all the action takes place in run()
         */
        explicit SerialLink(boost::asio::io_context& io_context);
        ~SerialLink();

        void send_threadsafe(rbl::IoBuffer::UPtr buffer_uptr);
        /**
         * SerialLink::run() tries to open the first device it finds where the path starts with
         * /dev/ttyACM and is a tty
         *
         * There after it reads from and writes to that device.
         *
         * If any that fails it sleeps for a second and then tries again forever.
         */
        [[noreturn]] void run(OnRecvCallback  cb);
    private:
        void start_read();
        void start_writes();
        boost::asio::io_context& m_io_context;
        std::deque<rbl::IoBuffer::UPtr> m_output_queue; 
        std::deque<std::unique_ptr<std::string>> m_output_queue_2; 

        OnRecvCallback                     m_recv_callback;
        
        int                 m_serial_fd;
        boost::asio::posix::stream_descriptor m_serial_stream;
        int                 m_output_queue_fd;
        int                 m_nbr_fds;

        rbl::IoBuffer::UPtr             m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written
        boost::asio::streambuf          m_read_streambuf;           // holds the databytes that have been read but not processed into messages
        rbl::IoBuffer::UPtr             m_input_message_buffer_uptr; // holds the bytes that have been processd from m_read_buffer into the
        
        void try_write();
        void try_read();
    };
} // namespace
#endif