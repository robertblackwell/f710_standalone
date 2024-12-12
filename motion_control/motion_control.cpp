#include <cmath>
#include <cassert>
#include <tuple>
#include "motion_control.h"

static IoBuffer::UPtr make_robot_message(int left, int right)
{
    auto left_percent = (float)round(((float)(left) / (float)INT16_MAX) * 100.0);
    auto right_percent = (float)round(((float)(right) / (float)INT16_MAX) * 100.0);
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}

using namespace non_ros_msgs;
using namespace rbl;
#ifdef CVQ_USE_HACK
void MotionControl::send_threadsafe(F710LeftRight::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::send_threadsafe(TwoEncoderStatus::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::send_threadsafe(FirmwareStartupResponse::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::run(const std::function<void(IoBuffer::UPtr iobuptr)>& msg_callback)
{
    while(true) {
        auto item = m_queue.get_wait();
        assert(item != nullptr);
        switch (item->type) {
            case 1: {
                int left = item->m_f710_uptr->m_left;
                int right = item->m_f710_uptr->m_right;
                delete item;
                auto iobu = make_robot_message(left, right);
                msg_callback(std::move(iobu));
            }
            break;
            case 2:
                RBL_LOG_FMT("motion_control got a two_encoder_status\n");
                delete item;
                break;
            case 3:
                RBL_LOG_FMT("motion_control got a firmware startup\n");
                delete item;
                break;
        }
    }
}
#else
void MotionControl::send_threadsafe(F710LeftRight::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::send_threadsafe(TwoEncoderStatus::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::send_threadsafe(FirmwareStartupResponse::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::run(const std::function<void(IoBuffer::UPtr iobuptr)>& buffer_callback)
{
    while(true) {
        auto item = m_queue.get_wait();
//        auto i = item->index();
        if(auto leftright = std::get_if<F710LeftRight::UPtr>(&*item)){
                auto lr = *(*leftright);
                auto iobu = make_robot_message(lr.m_left, lr.m_right);
                buffer_callback(std::move(iobu));
        
        } else if(auto ec = std::get_if<TwoEncoderStatus::UPtr>(&*item)) {
        
                RBL_LOG_FMT("motion_control got a two_encoder_status %ld\n", (*ec)->left.sample_sum);
                item = nullptr;
        
        } else if(auto fs = std::get_if<FirmwareStartupResponse::UPtr>(&*item)) {
        
                RBL_LOG_FMT("motion_control got a firmware startup %s\n", (*fs)->text.c_str());
                item = nullptr;
        
        } else {
            throw std::runtime_error("not one of the legal types");
        }   
    }
}
#endif


#include "motion_control.h"
