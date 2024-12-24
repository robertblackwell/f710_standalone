#ifndef F710_MOTION_CONTROL_H
#define F710_MOTION_CONTROL_H
#include <memory>
#include <variant>
#include <functional>
#include <rbl/iobuffer.h>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <rbl/logger.h>
#include <non_ros_messages/msg_struct.h>
#include <rbl/cv_queue.h>
using namespace non_ros_msgs;
using namespace rbl;
class MotionControl
{
public:
#define CVQ_USE_HACKx
#ifdef CVQ_USE_HACK
    struct HackUnion {
        int type;
        union {
            F710LeftRight::UPtr     m_f710_uptr;
            TwoEncoderStatus::UPtr  m_two_encoder_status_uptr;
            FirmwareStartupResponse::UPtr   m_firmware_startup;
        };
        explicit HackUnion(F710LeftRight::UPtr f710_uptr): type(1), m_f710_uptr(std::move(f710_uptr)){};
        explicit HackUnion(TwoEncoderStatus::UPtr two_encoders): type(2), m_two_encoder_status_uptr(std::move(two_encoders)){};
        explicit HackUnion(FirmwareStartupResponse::UPtr firmware_startup): type(3), m_firmware_startup(std::move(firmware_startup)){};
        ~HackUnion();
    };
#endif
    using QueueItemUPtrVariant = std::variant<F710LeftRight::UPtr, TwoEncoderStatus::UPtr, FirmwareStartupResponse::UPtr>;
    using QueueItemVariant = std::variant<F710LeftRight, TwoEncoderStatus, FirmwareStartupResponse>;
    using QueueItemVariantUPtr = std::unique_ptr<QueueItemVariant>;

    MotionControl()=default;
    void send_threadsafe(F710LeftRight::UPtr msg);
    void send_threadsafe(TwoEncoderStatus::UPtr msg);
    void send_threadsafe(FirmwareStartupResponse::UPtr msg);
    void run(std::function<void(IoBuffer::UPtr iobuptr)>);
private:
    ConditionVariableQueue<std::unique_ptr<QueueItemUPtrVariant>>   m_queue;

    // state machine variables
    std::function<void(IoBuffer::UPtr iobuptr)> m_buffer_callback;
    int m_state;
    int f710_target_throttle_left = 0;
    int f710_target_throttle_right = 0;
    double actual_left_throttle = 0.0;
    double actual_right_throttle = 0.0;
    double m_ratio = 0.0;
    double m_error = 0.0;
    int m_skip_encoder_status_count;

//    bool is_max_straight(F710LeftRight& lr);
//    bool is_max_forward(F710LeftRight& lr);
//    bool is_max_reverse(F710LeftRight& lr);
//    bool is_stop(F710LeftRight& lr);
    bool is_throttle_change(F710LeftRight& lr) const;
    /**
     * handle turn
        */
    void handle_turn_while_turning(F710LeftRight& lr);
    void handle_turn_while_going_straight(F710LeftRight& lr);
    void handle_turn_while_stopped(F710LeftRight& lr);
    /**
     * handle gostraight
     */
    void handle_gostraight_while_turning(F710LeftRight& lr);
    void handle_gostraight_while_going_straight(F710LeftRight& lr);
    void handle_gostraight_while_stopped(F710LeftRight& lr);
     /**
     * handle stop
     */
    void handle_stop_while_going_straight();
    void handle_stop_while_turning();
    /**
     * handle encoder update
     */
    void handle_encoder_update_while_going_straight(TwoEncoderStatus& ec);

    void handle_reboot_pico();
    void handle_pico_has_rebooted();


    void dump_encoder_status(TwoEncoderStatus& ec);




};

#endif //F710_MOTION_CONTROL_H
