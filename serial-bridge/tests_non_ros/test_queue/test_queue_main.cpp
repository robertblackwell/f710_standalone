
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <vector>
#include <string>
#include <unittest.h>
#include <queue.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <chrono>
#include <sys/select.h>
#include <sys/epoll.h>
#include <poll.h>
using namespace threadsafe;

int test_simple()
{
    printf("XXXXXXXXXXXHello world from buffer test \n");
    UT_EQUAL_INT(1, 1);
    Queue<std::string> queue;
    FdQueue<std::string> fdqueue;

    //prove read fd is non_blocking
    char buf[1000];
    int n = (int)read(fdqueue.read_fileno(), buf, 1000);
    UT_EQUAL_INT(n, -1)
    int errx = errno;
    UT_TRUE((errx == EAGAIN))
    std::string er{strerror(errx)};
    printf("%d", n);

    UT_TRUE(fdqueue.empty())

    return 0;
}
int test_plain_queue()
{
    threadsafe::Queue<std::string> test_queue{};
    int count = 0;

    std::jthread t([&](){
        while(true) {
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            auto now_c_str= std::ctime(&now_time);
            printf("%s",now_c_str);
            auto msg = std::string(now_c_str);
            test_queue.put(std::move(msg));
            sleep(2.0);
            if(++count == 10) {
                break;
            }
        }
    });
    while(true) {
        std::string msg{};
        test_queue.get(msg);
        printf("main thread got %s  count: %d\n", msg.c_str(), count);
        if(count >= 9) {
            break;
        }
    }
    t.join();
    return 0;
}
int test_fd_queue()
{
    threadsafe::FdQueue<std::string> test_queue{};
    int count = 0;

    auto writer = [&]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            auto now_c_str = std::ctime(&now_time);
            printf("background thread put count: %d msg: %s", count, now_c_str);
            auto msg = std::string(now_c_str);
            test_queue.put(std::move(msg));
            count++;
        }
    };
    auto reader_select = [&]() {
        while (true) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            fd_set write_fds;
            FD_ZERO(&read_fds);
            fd_set except_fds;
            FD_ZERO(&except_fds);
            timeval tv{};

            while (true) {
                FD_ZERO(&read_fds);
                FD_ZERO(&read_fds);
                FD_ZERO(&except_fds);

                int fd = test_queue.read_fileno();
                FD_SET(fd, &read_fds);
                int nbr_fds = fd + 1;
                int retval = select(nbr_fds, &read_fds, &write_fds, &except_fds, nullptr);
                if (retval < 0) {
                    int saved_errno = errno;
                    printf("select returned -1 errno is %d, %s", errno, strerror(errno));
                    throw std::runtime_error(
                            std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                        strerror(saved_errno)));
                } else {
                    std::string msg{};
                    bool r = FD_ISSET(fd, &read_fds);
                    bool w = FD_ISSET(fd, &write_fds);
                    bool x = FD_ISSET(fd, &except_fds);
                    if (r) {
                        bool result = test_queue.get_nowait(msg);
                        printf("main thread got :%s", msg.c_str());
                    }
                    printf("main thread r: %d w: %d x: %x\n", (int) r, (int) w, (int) x);
                }
            }
        }
    };
    auto reader_poll = [&]() {
        struct pollfd *pollfds;
        struct pollfd  read_pollfd = {.fd = test_queue.read_fileno(), .events = POLLIN, .revents = 0};
        pollfds = &(read_pollfd);
        nfds_t nfds = 1;
        int timeout = -1;

        while (true) {

            while (true) {

                int retval = poll(pollfds, nfds, timeout);
                if (retval < 0) {
                    int saved_errno = errno;
                    printf("select returned -1 errno is %d, %s", errno, strerror(errno));
                    throw std::runtime_error(
                            std::format("select error retval: %d  errno: %d  strerror: %s", retval, saved_errno,
                                        strerror(saved_errno)));
                } else {
                    std::string msg{};
                    bool r = read_pollfd.revents & POLLIN;
                    bool w = read_pollfd.revents & POLLOUT;
                    bool x = read_pollfd.revents & ~POLLIN & ~POLLOUT;
                    if (r) {
                        bool result = test_queue.get_nowait(msg);
                        printf("main thread got :%s", msg.c_str());
                    }
                    printf("main thread revent: %x   r: %d w: %d x: %x\n", read_pollfd.events, (int) r, (int) w, (int) x);
                }
            }
        }
    };



    std::jthread t(reader_poll);
//    std::this_thread::sleep_for(std::chrono::milliseconds(500) );
    writer();
    t.join();
    return 0;
}


int main()
{
    UT_ADD(test_fd_queue);
    int rc = UT_RUN();
    return rc;
}