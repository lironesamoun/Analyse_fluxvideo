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
    Timer();

public:
    void startTimer();
    void stopTimer();
    long getTime();
    void startTimerFPS();
    void stopTimerFPS();
    double getFPS();


    timeval m_start;
    timeval m_end;
    time_t start, end;
    double fps;
    int counter;
    double sec;

};//Timer


}

#endif //DRONE_TIMER_HPP

