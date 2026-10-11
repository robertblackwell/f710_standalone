// #include "Arduino.h"
#define FTRACE_ON
#include "commands.h"
#include <stdlib.h>
#include "argv.h"
#include "trace.h"

#define CLI_STATE_START 1
#define CLI_STATE_READING_LINE 2
#define CLI_STATE_COMMAND_AVAILABLE 3

#define CLI_STATE_NEED_ARGS 4

int atpwm(char* p);
bool ispwmvalue(const char* p);
bool isnegnum(const char* p);
bool isposnum(const char* p);

int atpwm(char* p) {
    return atoi(p);
}

bool ispwmvalue(const char* p) {
    if(isnegnum(p) || isposnum(p)) {
        int v = atoi(p);
        return (v >= -255) && (v <= 255);
    }
    return false;
}


bool isnegnum(const char* p) {
    if(*p == '-') {
        return isposnum(&p[1]);
    }
    return false;
}

bool isposnum(const char* p) {
    FTRACE("isposnum %s\n", p);
    const char* pp = p;
    while(*pp != (char)0) {
        if(!isdigit(*pp)) {
            FTRACE("isposnum %s %d\n", p, 0);
            return false;
        }
        pp++;
    }
    FTRACE("isposnum %s %d\n", p, 1);
    return true;
}
bool isfloat(const char* p) {
    
    const char* q = p;
    if(*q == '\0') {
        return false;
    }
    if(*q == '-') {
        q++;
    }
    if(*q == '\0') {
        return false;
    }
    while(isdigit(*q)) {
        q++;
    }
    if(*q == '\0') {
        return true;
    } else if(*q == '.') {
        q++;
    } else {
        return  false;
    }
    while(isdigit(*q)) {
        q++;
    }
    if(*q != '\0') {
        return false;
    }
    return true;
}
bool isMotorDirection(char* s) {
    if(strlen(s) == 1) {
        if((s[0] == 't') || (s[0] == 'f')) {
            FTRACE("isMotorDirection [%s] %d\n", s, 0);
            return true;
        }
    }
    FTRACE("isMotorDirection [%s] %d\n", s, 0);
    return false;
}

bool validate_float(const char* p, float* value) {
    if(isfloat(p)) {
        float v = atof(p);
        *value = v;
        return true;
    }
    return false;
}
bool validate_double(const char* p, double* value) {
    if(isfloat(p)) {
        double v = atof(p);
        *value = v;
        return true;
    }
    return false;
}

bool validate_pwm(const char* p, int* pwm_value) {
    if(ispwmvalue(p)) {
        *pwm_value = atoi(p);
        return true;
    }
    return false;
}

bool validate_two_pwm(Argv& args, int *arg1, int * arg2) {
    return validate_pwm(args.token_at(1), arg1) && validate_pwm(args.token_at(2), arg2);
}

bool validate_two_float(Argv& args, float *arg1, float * arg2) {
    return validate_float(args.token_at(1), arg1) && validate_float(args.token_at(2), arg2);
}
bool validate_two_double(Argv& args, double *arg1, double * arg2) {
    return validate_double(args.token_at(1), arg1) && validate_double(args.token_at(2), arg2);
}

bool validate_three_float(Argv& args, float *arg1, float * arg2, float* arg3) {
    return validate_float(args.token_at(1), arg1)
        && validate_float(args.token_at(2), arg2)
        && validate_float(args.token_at(3), arg3);
}
bool validate_three_double(Argv& args, double *arg1, double * arg2, double* arg3) {
    return validate_double(args.token_at(1), arg1)
        && validate_double(args.token_at(2), arg2)
        && validate_double(args.token_at(3), arg3);
}

bool validate_pwm(Argv& args, double& left, double& right)
{
    bool valid = (args.token_count == 3) 
                && argparse_double(args, 1, left) 
                && argparse_double(args, 2, right) 
                && (
                    ((-100.0 <= left) && (left <= 100.0))
                ) && (
                    ((-100.0 <= right) && (right <= 100.0))
                );
    return valid;  
}
bool validate_rpm(Argv& args, double& left, double& right)
{
    bool valid = (args.token_count == 3)
                && argparse_double(args, 1, left) 
                && argparse_double(args, 2, right) 
                && (
                    ((-8000.0 <= left) && (left <= 0.0)) || ((0.0 <= left) && (left <= 8000.0))
                ) && (
                    ((-8000.0 <= right) && (right <= 0.0)) || ((0.0 <= right) && (right <= 8000.0))
                );
    return valid;  
}
bool validate_wheel_velocity(Argv& args, double& left, double& right)
{
    bool valid = (args.token_count == 3)
                && argparse_double(args, 1, left)
                && argparse_double(args, 2, right);
                // && (
                //     ((-8000.0 <= left) && (left <= 0.0)) || ((0.0 <= left) && (left <= 8000.0))
                // ) && (
                //     ((-8000.0 <= right) && (right <= 0.0)) || ((0.0 <= right) && (right <= 8000.0))
                // );
    return valid;
}
bool validate_stop(Argv& args)
{
    bool valid = (args.token_count >= 1);
    return valid;  
}
bool validate_echo(Argv& args)
{
    bool valid = (args.token_count >= 1);
    return valid;  
}
bool validate_encoder_read(Argv& args)
{
    bool valid = (args.token_count >= 1);
    return valid;  
}
bool validate_encoders_stream(Argv& args, int& interval_ms )
{
    bool valid = (args.token_count == 2)
            && argparse_posint(args, 1, interval_ms);
    return valid;  
}
bool validate_loadtest(Argv& args, int& count, int& length, int& num_per_second)
{
    bool valid = (args.token_count == 4)
                && argparse_posint(args, 1, count)
                && argparse_posint(args, 2, length)
                && argparse_posint(args, 3, num_per_second);
    return valid;
}
CommandName command_lookup(const char* first_arg) {
    struct TableEntry {
        const char* short_name;
        const char* long_name;
        int         tag;
        CommandName cmd;
    };
    static TableEntry table[] = {
        {.short_name = "c",  .long_name = "echo",       .tag = CLI_COMMAND_TAG_ECHO,            .cmd = CommandName::Echo},
        {.short_name = "s",  .long_name = "stop",       .tag = CLI_COMMAND_TAG_STOP,            .cmd = CommandName::MotorsHalt},
        {.short_name = "w",  .long_name = "pwm",        .tag = CLI_COMMAND_MOTOR_PERCENT_PWM,   .cmd = CommandName::MotorsPwmPercent},
        {.short_name = "v",  .long_name = "vw",         .tag = CLI_COMMAND_WHEEL_VELOCITY,      .cmd = CommandName::WheelVelocity},
        {.short_name = "vs", .long_name = "vwscaled",   .tag = CLI_COMMAND_WHEEL_VELOCITY,      .cmd = CommandName::WheelVelocityScaled},

        {.short_name = "m",  .long_name = "motor",      .tag = CLI_COMMAND_MOTOR_SPEED,         .cmd = CommandName::MotorsRpm}, // rpw
        {.short_name = "r",  .long_name = "rpm",        .tag = CLI_COMMAND_MOTOR_RPM,           .cmd = CommandName::MotorsRpm},

        {.short_name = "t",  .long_name = "stop",       .tag = CLI_COMMAND_TAG_STOP,            .cmd = CommandName::MotorsHalt},

        {.short_name = "u",  .long_name = "upid",       .tag = CLI_COMMAND_UPDATE_PIDARGS,      .cmd = CommandName::PidArgsUpdate},
        {.short_name = "e",  .long_name = "encoder",    .tag = CLI_COMMAND_READ_ENCODERS,       .cmd = CommandName::EncodersRead},
        {.short_name = "es", .long_name = "encoder",    .tag = CLI_COMMAND_READ_ENCODERS,       .cmd = CommandName::EncodersStream},
        {.short_name = "l",  .long_name = "load",       .tag = CLI_COMMAND_TAG_LOADTEST,        .cmd = CommandName::LoadTest},
        {.short_name = "?",  .long_name = "help",       .tag = CLI_COMMAND_TAG_LOADTEST,        .cmd = CommandName::Help},
        {.short_name = "b",  .long_name = "reset",      .tag = CLI_COMMAND_TAG_NONE,            .cmd = CommandName::SoftwareReset},
        {.short_name = "p",  .long_name = "pid",        .tag = CLI_COMMAND_PID_ONOFF,           .cmd = CommandName::PidOnOff},

        /* the next line must be at the end of the table */
        {.short_name = "x",  .long_name = "raw",    .tag = CLI_COMMAND_TAG_NONE,            .cmd = CommandName::None},

    };
    // log_print("lookup ", first_arg, "\n");
    for(int i = 0; *(table[i].short_name) != 'x'; i++) {
        // log_print("lookup loop first_arg: ", first_arg, " i:",i, " ", table[i].short_name, ' ', table[i].long_name, "\n");
        if((strcmp(first_arg, table[i].short_name) == 0) || (strcmp(first_arg, table[i].long_name) == 0)) {
            // log_print("lookup ", first_arg, " found tag:", table[i].tag,  "\n");
            return table[i].cmd;
        }
    }
    // log_print("lookup failed", first_arg, " found tag:", CLI_COMMAND_TAG_NONE,  "\n");
    return CommandName::Error;
}
bool deserialize(Argv& args, pico_messages::CommandMessageUnion& msg)
{
    CommandName name = command_lookup(args.token_at(0));
    switch (name) {
        case CommandName::MotorsPwmPercent:{
            double pwm_percent_left;
            double pwm_percent_right;
            if((args.token_count == 3) && validate_two_double(args, &pwm_percent_left, &pwm_percent_right)) {
                msg.motor_pwm_cmd.left_motor_pwm = pwm_percent_left;
                msg.motor_pwm_cmd.right_motor_pwm = pwm_percent_right;
                msg.tag = pico_messages::CommandMessageEnum::motor_pwm_cmd;
            } else {
                return false;
            }
            break;
        }
        case CommandName::MotorsRpm:{
            double rpm_left;
            double rpm_right;

            if((args.token_count == 3) && validate_two_double(args, &rpm_left, &rpm_right)) {
                msg.tag = pico_messages::CommandMessageEnum::motor_rpm_cmd;
                msg.motor_rpm_cmd.left_motor_rpm = rpm_left;
                msg.motor_rpm_cmd.right_motor_rpm = rpm_right;
            } else {
                return false;
            }
            break;
        }
        case CommandName::MotorsHalt:
            msg.tag = pico_messages::CommandMessageEnum::stop_cmd;
            break;
        case CommandName::PidOnOff:
            msg.tag = pico_messages::CommandMessageEnum::pid_toggle_cmd;
            break;
        case CommandName::WheelVelocity:{
            double velocity_left;
            double velocity_right;

            if((args.token_count == 3) && validate_two_double(args, &velocity_left, &velocity_right)) {
                msg.tag = pico_messages::CommandMessageEnum::wheel_velocity_cmd;
                msg.wheel_velocity_cmd.left_wheel_velocity_ms = velocity_left;
                msg.wheel_velocity_cmd.right_wheel_velocity_ms = velocity_right;
            } else {
                return false;
            }
            break;
    }
    case CommandName::WheelVelocityScaled:{
            double left;
            double right;

            if((args.token_count == 3) && validate_two_double(args, &left, &right)) {
                msg.tag = pico_messages::CommandMessageEnum::wheel_velocity_scaled_cmd;
                msg.wheel_velocity_scaled_cmd.left_wheel_velocity_percent = left;
                msg.wheel_velocity_scaled_cmd.right_wheel_velocity_percent = right;
            } else {
                return false;
            }
            break;
    }
    case CommandName::PidArgsUpdate:{
            double kp;
            double ki;
            double kd;

            if((args.token_count == 4) && validate_three_double(args, &kp, &ki, &kd)) {
                msg.tag = pico_messages::CommandMessageEnum::pid_args_update_cmd;
                msg.pid_args_update_cmd.kp = kp;
                msg.pid_args_update_cmd.ki = ki;
                msg.pid_args_update_cmd.kd = kd;
            } else {
                return false;
            }
            break;
        }
        case CommandName::EncodersRead:{
            if((args.token_count == 2) && (isposnum(args.token_at(1)) || isnegnum(args.token_at(1)))) {
                msg.tag = pico_messages::CommandMessageEnum::read_encoders_cmd;
                msg.read_encoders_cmd.how_many = atoi(args.token_at(1));
            } else if(args.token_count == 1) {
                msg.tag = pico_messages::CommandMessageEnum::read_encoders_cmd;
                msg.read_encoders_cmd.how_many = 1;
            } else {
                return false;
            }
            break;
        }
        case CommandName::EncodersStream:{
            if((args.token_count == 3) && (isposnum(args.token_at(1)) && isposnum(args.token_at(2)))) {
                msg.tag = pico_messages::CommandMessageEnum::stream_encoders_cmd;
                msg.stream_encoders_cmd.how_long_between_ms = atoi(args.token_at(1));
                msg.stream_encoders_cmd.how_many = atoi(args.token_at(2));
            } else {
                return false;
            }
            break;
        }
        case CommandName::Echo: {
            strcpy(&echo_command.line[0], &(args.m_unmodified_line_buffer[0]));
            break;
        }
        default: {
            m_command_enum = CommandName::Error;
            error_command.create_error_msg("Unknown command", tokens.m_unmodified_line_buffer);
            return false;
            break;
        }
    }
    return true;
}
bool deserialize(char* line, pico_messages::CommandMessageUnion& msg)
{
    Argv args{line};
    return deserialize(args, msg);
}
void CommandBuffer::copyTo(CommandBuffer& to)
{
    
}
CommandName CommandBuffer::identity()
{
    return m_command_enum;
}
#endif