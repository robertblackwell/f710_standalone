#ifndef H_port_h
#define H_port_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>

using namespace rbl;
namespace serial_bridge {
    class SerialLink {
    public:
    typedef std::function<void(IoBuffer::UPtr)>  OnRecvCallback;
    typedef std::unique_ptr<SerialLink>          UPtr; 
        /**
         * The constructor only initializes a few member variables.
         * all the action takes place in run()
         */
        explicit SerialLink();
        ~SerialLink();

        void send_threadsafe(IoBuffer::UPtr buffer_uptr) const;
        /**
         * SerialLink::run() tries to open the first device it finds where the path starts with
         * /dev/ttyACM and is a tty
         *
         * There after it reads from and writes to that device.
         *
         * If any that fails it sleeps for a second and then tries again forever.
         */
        void run(OnRecvCallback  cb);

#if 0
//        rclcpp::Logger get_logger();
        void guard_condition_callback(std::size_t n);
//        void guard_trigger_function();
//        rclcpp::Node*                      m_companion_node_ptr;
//        rclcpp::GuardCondition::SharedPtr  m_guard_condition_sptr;
#endif
        std::unique_ptr<threadsafe::FdQueue<IoBuffer::UPtr>>      m_output_queue_uptr;
        std::unique_ptr<threadsafe::TriggerQueue<IoBuffer::UPtr>> m_client_queue_uptr;

        OnRecvCallback                     m_recv_callback;
        
        int                 m_serial_fd;
        int                 m_output_queue_fd;
        fd_set              m_rfds;
        fd_set              m_wfds;
        fd_set              m_xfds;
        int                 m_nbr_fds;
        struct timeval      tv;
        bool                m_read_eagained_flag;

        IoBuffer::UPtr      m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written
        IoBuffer::UPtr      m_read_buffer_uptr;         // holds the databytes that have been read but not processed into messages
        IoBuffer::UPtr      m_input_message_buffer_uptr; // holds the bytes that have been processd from m_read_buffer into the
        
        void try_write();
        void try_read();
    };
} // namespace
#endif