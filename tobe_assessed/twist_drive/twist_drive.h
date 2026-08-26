#ifndef F710_twist_drive_H
#define F710_twist_drive_H
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
namespace twist_drive {
    enum class TwistCategory{
        TC_STOP,
        TC_FORWARD_STRAIGHT,
        TC_REVERSE_STRAIGHT,
        TC_FORWARD_TURNING_RIGHT,
        TC_FORWARD_TURNING_LEFT,
        TC_REVERSE_TURNING_RIGHT,
        TC_REVERSE_TURNING_LEFT,
        TC_TRANSITION_TURNING_TO_GS,
    };
    struct TwistCommand {
        TwistCategory  category;
        long           left;
        long           right;
        double         angular_vel;
        double         robot_vel;
        double         left_wheel_vel;
        double         right_wheel_vel;
        double         left_pwm;
        double         right_pwm;
    };

    enum class TwistState {
        STARTUP,
        WAITING_FOR_PICO_TO_BOOT,
        STATIONARY,
        FORWARD_STRAIGHT,
        REVERSE_STRAIGHT,
        FORWARD_TURNING_RIGHT,
        FORWARD_TURNING_LEFT,
        REVERSE_TURNING_RIGHT,
        REVERSE_TURNING_LEFT,
        TRANSITION_TURNING_TO_GS

    };

    class TwistDrive {
    public:
#define CVQ_USE_HACK
#ifdef CVQ_USE_HACK
        struct HackUnion {
            enum class Type{LeftRight, EncoderStatus, FirmwareStartup};
            Type type;
            union {
                F710LeftRight::UPtr             m_f710_uptr;
                TwoEncoderStatus::UPtr          m_two_encoder_status_uptr;
                FirmwareStartupResponse::UPtr   m_firmware_startup;
            };
            explicit HackUnion(F710LeftRight::UPtr f710_uptr): type(Type::LeftRight), m_f710_uptr(std::move(f710_uptr)){};
            explicit HackUnion(TwoEncoderStatus::UPtr two_encoders): type(Type::EncoderStatus), m_two_encoder_status_uptr(std::move(two_encoders)){};
            explicit HackUnion(FirmwareStartupResponse::UPtr firmware_startup): type(Type::FirmwareStartup), m_firmware_startup(std::move(firmware_startup)){};
            ~HackUnion();
        };
        using QueueItemUPtrVariant = HackUnion;
#else
        using QueueItemUPtrVariant = std::variant<F710LeftRight::UPtr, TwoEncoderStatus::UPtr, FirmwareStartupResponse::UPtr>;
        using QueueItemVariant = std::variant<F710LeftRight, TwoEncoderStatus, FirmwareStartupResponse>;
        using QueueItemVariantUPtr = std::unique_ptr<QueueItemVariant>;
#endif
        TwistDrive() = default;

        void send_threadsafe(F710LeftRight::UPtr msg);

        void send_threadsafe(TwoEncoderStatus::UPtr msg);

        void send_threadsafe(FirmwareStartupResponse::UPtr msg);

        void run(std::function<void(IoBuffer::UPtr iobuptr)>);

    private:
        TwistCommand f710LeftRightToTwist(F710LeftRight::UPtr lr);
        void handle_firmware_startup();
        void handle_leftright();
        void handle_encoder_status();

        ConditionVariableQueue<std::unique_ptr<QueueItemUPtrVariant>> m_queue;

        // state machine variables
        std::function<void(IoBuffer::UPtr iobuptr)> m_buffer_callback;
        TwistState m_state;
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
        bool is_throttle_change(F710LeftRight &lr) const;

        /**
         * handle turn
            */
        void handle_turn_while_turning(F710LeftRight &lr);

        void handle_turn_while_going_straight(F710LeftRight &lr);

        void handle_turn_while_stopped(F710LeftRight &lr);

        /**
         * handle gostraight
         */
        void handle_gostraight_while_turning(F710LeftRight &lr);

        void handle_gostraight_while_going_straight(F710LeftRight &lr);

        void handle_gostraight_while_stopped(F710LeftRight &lr);

        /**
        * handle stop
        */
        void handle_stop_while_going_straight();

        void handle_stop_while_turning();

        /**
         * handle encoder update
         */
        void handle_encoder_update_while_going_straight(TwoEncoderStatus &ec);

        void handle_reboot_pico();

        void handle_pico_has_rebooted();


        void dump_encoder_status(TwoEncoderStatus &ec);


    };
} //namespace
#endif //F710_MOTION_CONTROL_H
