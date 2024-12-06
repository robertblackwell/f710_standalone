#ifndef F710_OUTPUTTER_H
#define F710_OUTPUTTER_H
#include <string>
#include <bridge_lib/queue.h>
#include <bridge_lib/iobuffer.h>
#include "cv_queue.h"

class Outputter
{
public:
    Outputter();
    ~Outputter();
    void send_threadsafe(std::string msg);
    void run();
private:
    ConditionVariableQueue<std::string> m_queue;
};


#endif //F710_OUTPUTTER_H
