#define GNU_SOURCE
#include <stdio.h>
#include <thread>
#include <climits>
#include <f710.h>
#include <rbl/std_format.h>
#include <rbl/iobuffer.h>
#include <bridge_lib/serial_link.h>
#include <bridge_lib/serial_settings.h>
#include <rbl/simple_exit_guard.h>
#include "outputter.h"



using namespace serial_bridge;
using namespace f710;

int main()
{

    try {
        SerialLink serial_link{};
        Outputter  outputter{};

        /** BEWARE this function is called on the tserial thread*/
        SerialLink::OnRecvCallback on_incoming_message = [&](IoBuffer::UPtr iob_uptr) -> void {
            std::string msg = std::string(iob_uptr->get_first_char_ptr());
            iob_uptr = nullptr;
            outputter.send_threadsafe(msg);
        };
        std::thread toutputter{[&]() -> void { outputter.run(); }};
        std::thread tserial{[&]() -> void { serial_link.run(on_incoming_message); }};
        tserial.join();
        toutputter.join();
        printf("after all joins \n");
    } catch(...) {
        printf("main catch \n");
    };

	printf("after serial thread start about to spin\n");
	return 0;
}

