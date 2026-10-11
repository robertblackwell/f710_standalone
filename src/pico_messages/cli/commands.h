#ifndef H_commands_h
#define H_commands_h
#include <pico_messages/msg_struct.h>
#include "argv.h"

#define CLI_COMMAND_TAG_NONE            0   // an illegal command
#define CLI_COMMAND_TAG_ERROR           4   // used internally only for commands that do not parse correctly 

#define CLI_COMMAND_TAG_SPEED           1   // sets the pwm percentage value for both motors 
                                            //  -   command has 2 arguments left-percentage-pwm right-percentage-
                                            //  -   these are floating point numbers in the range -100.00 to 100.00
                                            //  -   these are convered into 4 values for the command arg structure
#define CLI_COMMAND_MOTOR_PERCENT_PWM   11  // this is the similar to the speed command except direction is given in the sign of the pwm value
                                            // 2 arguments each are floating point numbers -100.00 .. 100.00

#define CLI_COMMAND_MOTOR_RPM           5   // set the speed of motors to an rpm value - two arguments 3500-7500
#define CLI_COMMAND_MOTOR_SPEED         10  // set the speed of each motor in rpm
#define CLI_COMMAND_MOTOR_RPM           5   // set the speed of motors to an rpm value - two arguments 3500-7500
#define CLI_COMMAND_WHEEL_VELOCITY      15  // set the speed of each wheel in meters per sec
#define CLI_COMMAND_WHEEL_VELOCITY_SCALED      16  // set the speed of each wheel in -100.0 .. 100.0

#define CLI_COMMAND_TAG_STOP            2   // no arguments

#define CLI_COMMAND_UPDATE_PIDARGS      7   // update the parameters of the embedded PID algorithm

#define CLI_COMMAND_READ_ENCODERS       8   // read the latest value of the encoders
#define CLI_COMMAND_RESET_ENCODERS      9   // reset the encoder recording of ticks 
#define CLI_COMMAND_TAG_ECHO            10
#define CLI_COMMAND_TAG_LOADTEST        11
#define CLI_COMMAND_PID_ONOFF           14


enum class CommandName
{
    None = 'n',
    Error = 'x',
    MotorsPwmPercent = 's',
    MotorsRpm = 'r',
    WheelVelocity = 'v',
    WheelVelocityScaled = 'y',
    MotorsHalt = 'h',
    PidArgsUpdate = 'u',
    EncodersRead = 'e',
    EncodersStream = 't',
    Echo = 'c',
    LoadTest = 'l',
    Help = '?',
    SoftwareReset = 'b',
    PidOnOff = 'p',
};
inline const char* to_string(CommandName en)
{
    const char* s = "Unknown";;
    switch(en) {
        case CommandName::Echo:
            s = "Echo";
            break;
        case CommandName::None:
            s = "None";
            break;
        case CommandName::Error:
            s =  "Error";
            break;
        case CommandName::MotorsPwmPercent:
            s =  "MotorsPwmPercent";
            break;
        case CommandName::MotorsRpm:
            s =  "MotorsRpm";
            break;
        case CommandName::WheelVelocity:
            s =  "WheelVelocity";
            break;
        case CommandName::WheelVelocityScaled:
            s =  "WheelVelocityScaled";
            break;
        case CommandName::MotorsHalt:
            s =  "MotorsHalt";
            break;
        case CommandName::PidArgsUpdate:
            s =  "PidArgsUpdate";
            break;
        case CommandName::EncodersRead:
            s =  "EncodersRead";
            break;
        case CommandName::EncodersStream:
            s =  "EncodersStream";
            break;
        case CommandName::LoadTest:
            s = "Loadtest";
            break;
        case CommandName::Help:
            s = "Help";
            break;
        case CommandName::SoftwareReset:
            s = "SoftwareReset";
            break;
        case CommandName::PidOnOff:
            s = "PidOnOff";
    }
    return s;
}
 
CommandName command_lookup(const char* first_arg);
bool validate_pwm(Argv& args, double& left, double& right);
bool validate_rpm(Argv& args, double& left, double& right);
bool validate_wheel_velocity(Argv& args, double& left, double& right);
bool validate_stop(Argv& args);
bool validate_echo(Argv& args);
bool validate_encoder_read(Argv& args);
bool validate_encoders_stream(Argv& args, int& interval_ms);
bool validate_loadtest(Argv& args, int& count, int& length, int& num_per_second);
bool validate_help(Argv& args);
bool deserialize(char* line, pico_messages::CommandMessageUnion& msg);
bool deserialize(Argv& args, pico_messages::CommandMessageUnion& msg);
#endif