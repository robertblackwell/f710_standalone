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
    void run(const std::function<void(IoBuffer::UPtr iobuptr)>&);
private:
#ifdef CVQ_USE_HACK
    ConditionVariableQueue<HackUnion*>   m_queue;
#else
    ConditionVariableQueue<std::unique_ptr<QueueItemUPtrVariant>>   m_queue;
#endif
};

#endif //F710_MOTION_CONTROL_H
