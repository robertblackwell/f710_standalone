#ifndef H_pico_serialize_commands_h
#define H_pico_serialize_commands_h
#include <string>
#include <variant>
#include <optional>
#include "rbl/iobuffer.h"
#include "msg_struct.h"

using namespace rbl;
namespace pico_messages {
    void serialize(CommandMessageUnion& msg, IoBuffer& buffer);
} // namespace pico_messages
#endif