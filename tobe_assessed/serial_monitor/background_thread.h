#ifndef GTKTEST_BACKGROUND_THREAD_H
#define GTKTEST_BACKGROUND_THREAD_H
#include <gtk/gtk.h>
#include <string>
#include <mutex>
#include <queue>
#include <rbl/iobuffer.h>
#include <bridge_lib/serial_link.h>
//#include "app_context.h"

struct BackgroundContext
{
    int 				    fd_read;
    int                     fd_write;
    std::mutex  		    mux;
    std::queue<std::string*> q;
    pthread_t               background_thread;
    GIOChannel*             background_channel;
    explicit BackgroundContext(): fd_read(-1), fd_write(-1), background_channel(nullptr)
    {

    }
};

void background_init();
void background_start_thread(GIOFunc  mainthread_on_message);
void background_join();
void background_send_threadsafe(std::string* command);
void background_send_threadsafe(rbl::IoBuffer::UPtr iobuptr);
std::string* background_get_message(BackgroundContext* bc);
void q_put(BackgroundContext* bc, std::string* msg);
std::string* q_get(BackgroundContext* bc);
BackgroundContext* get_background_context();

#endif //GTKTEST_BACKGROUND_THREAD_H
