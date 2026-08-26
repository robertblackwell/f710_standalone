//
// Created by robert on 12/22/24.
//
#include <gtk/gtk.h>
#include <pthread.h>
#include <exception>
//#include "app_context.h"
#include "background_thread.h"

[[noreturn]] void* thread_fn(void* arg);
GIOChannel* background_channel;
BackgroundContext background_context;
serial_bridge::SerialLink      serial_link{};

void background_init()
{
    int pipefd[2];
    int status = pipe(pipefd);
    if(status != 0) {
        throw std::runtime_error("ppe call failed");
    }
    background_context.fd_read = pipefd[0];
    background_context.fd_write = pipefd[1];
}
void background_start_thread(GIOFunc  mainthread_on_message)
{

    background_context.background_channel = g_io_channel_unix_new(background_context.fd_read);
    gint priority = 1;
    g_io_add_watch_full(background_context.background_channel, priority, G_IO_IN, mainthread_on_message, (gpointer)&background_context, nullptr);
    pthread_create(&(background_context.background_thread), nullptr, thread_fn, (void*)&background_context);
}
void background_join()
{
    pthread_join((background_context.background_thread), nullptr);
}
std::string* background_get_message(BackgroundContext* bc)
{
    auto msg = q_get(bc);
    if(!msg) {
        throw std::runtime_error("nullptr from interthread queue");
    }
    printf("message callback  msg_ptr:%p msg:%s  \n", (void*)msg, msg->c_str());
    char buf[10];
    long n = read(bc->fd_read, buf, 1);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    auto saved_errno = errno;
#pragma GCC diagnostic pop
    if(n < 0) {
        throw std::runtime_error("read call failed");
    }
    return msg;
}
void background_send_threadsafe(std::string* command) {

    auto iob_uptr = std::make_unique<IoBuffer>(*command);
    delete command;
    background_send_threadsafe(std::move(iob_uptr));
}
void background_send_threadsafe(rbl::IoBuffer::UPtr iob_uptr)
{
    serial_link.send_threadsafe(std::move(iob_uptr));
}
[[noreturn]] void* thread_fn(void* arg)
{
    auto wptr = (BackgroundContext*)arg;
    serial_link.run([&](rbl::IoBuffer::UPtr iob_uptr)  {
        const char* sig_buf = "XXXXXXXX";
        auto msg = new std::string(iob_uptr->c_str());
        q_put(wptr, msg);
        write(wptr->fd_write, (void*)sig_buf, 1);
    });
#if 0
    int msg_count = 0;
    while(true) {
        sleep(2);
        pthread_t id = pthread_self();
        g_print("worker thread id: %ld\n", id);
        const char* sig_buf = "XXXXXXXX";
        char* buf;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
        auto x = asprintf(&buf, "This is a message from worker %d", msg_count++);
#pragma GCC diagnostic pop
        auto msg = new std::string(buf);
        free(buf);
        q_put(wptr, msg);
        write(wptr->fd_write, (void*)sig_buf, 1);
    }
#endif
}
void q_put(BackgroundContext* bc, std::string* msg)
{
    std::lock_guard<std::mutex> guard(bc->mux);
    bc->q.push(msg);
}
std::string* q_get(BackgroundContext* bc)
{
    std::string* item = nullptr;
    std::lock_guard<std::mutex> guard(bc->mux);
    if(!bc->q.empty()) {
        item = bc->q.front();
        bc->q.pop();
    }
    return item;
}
BackgroundContext* get_background_context()
{
    return &background_context;
}
