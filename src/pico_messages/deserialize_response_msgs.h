#ifndef H_pico_response_messages_deserialize_h
#define H_pico_response_messages_deserialize_h
#include "rbl/iobuffer.h"
#include "msg_struct.h"

using namespace rbl;
namespace pico_messages {
    bool deserialize(IoBuffer& buffer, ResponseMessageUnion& inmsg);
    ResponseMessageUnion::UPtr deserialize(IoBuffer& buffer);
}
#endif