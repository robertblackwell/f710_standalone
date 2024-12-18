#include <rbl/cv_queue.h>
#include <cstdio>
#include <thread>
#include <string>
#include <rbl/std_format.h>
#include <pthread.h>

typedef std::string QueueItem;

ConditionVariableQueue<std::string> queue;

int producer()
{
    for(int i = 0; i < 10; i++) {
        auto id = gettid();
        auto s = std_format("from producer {}  count: {}", id, i);
        queue.put(s);
    }

    return 0;
}
int consumer()
{
    int i = 0;
    auto tid = gettid();
    while(true) {
        auto s = queue.get_wait();
        std::cout << "consumer " << tid << " got: " << s << std::endl;
        i++;
    }
    return 0;
}
int main()
{
    std::thread c1{[](){
        consumer();
    }};
//    std::thread c2{[](){
//        consumer();
//    }};
    std::thread p1{[](){
        producer();
    }};
    std::thread p2{[](){
        producer();
    }};
    std::thread p3{[](){
        producer();
    }};
    c1.join();
//    c2.join();
    p1.join();
    p2.join();
    p3.join();
}