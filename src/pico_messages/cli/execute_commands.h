//
// Created by robert on 3/11/24.
//

#ifndef PICO_ROBOT_EXECUTE_COMMANDS_H
#define PICO_ROBOT_EXECUTE_COMMANDS_H
#include "cli/argv.h"
#include "transport/buffers.h"

void execute_commands(Argv& args, transport::buffer::Handle bh);

#endif //PICO_ROBOT_EXECUTE_COMMANDS_H
