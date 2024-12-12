#include "outputter.h"

Outputter::Outputter() = default;
Outputter::~Outputter() = default;

void Outputter::send_threadsafe(std::string msg)
{
    m_queue.put(msg);
}

void Outputter::run()
{
    while(true) {
        auto s = m_queue.get_wait();
        std::cout << "From outputter: " << s << std::endl;
    }
}