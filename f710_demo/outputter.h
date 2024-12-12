#ifndef F710_OUTPUTTER_H
#define F710_OUTPUTTER_H
#include <string>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>
#include <rbl/cv_queue.h>

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
