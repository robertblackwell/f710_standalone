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

static IoBuffer::UPtr make_robot_message(int left, int right);
static std::string make_outputter_message(int left, int right);

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

static IoBuffer::UPtr make_robot_message(int left, int right)
{
    auto left_percent = (float)round(((float)(left) / (float)INT16_MAX) * 100.0);
    auto right_percent = (float)round(((float)(right) / (float)INT16_MAX) * 100.0);
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
static std::string make_outputter_message(int left, int right)
{
    char* bufptr; 
    asprintf(&bufptr, "left: %d  right: %d", left, right);
    auto s = std::string(bufptr);
    free(bufptr);
    return s;
//    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
//    int len = snprintf(iob->get_first_char_ptr(), 256, "received %d %d  ", left, right);
//    iob->setSize(len);
//    return iob;
}
