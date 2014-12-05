#ifndef DRONE_TIMER_HPP
#define DRONE_TIMER_HPP

#include "sys/time.h"
#include <cstdlib>
#include <string>
#include <vector>
#include "fstream"
#include "iostream"
#include <sstream>

namespace drone
{

class Timer
{
public:
    void startTimer();
    void stopTimer();
    long getTime();

private:
    timeval m_start;
    timeval m_end;

};//Timer


}

#endif //DRONE_TIMER_HPP

