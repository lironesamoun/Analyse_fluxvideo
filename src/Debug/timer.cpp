#include "timer.hpp"
#include "math.h"

namespace drone
{

Timer::Timer(){
    this->counter=0;
}

void Timer::startTimerFPS(){
     time(&start);
}

void Timer::stopTimerFPS(){
    time(&end);
    ++counter;
    sec = difftime (end, start);
    fps = counter / sec;

}

double Timer::getFPS(){
       std::cout << "current fps: " << fps << std::endl;
}

void Timer::startTimer(){
    gettimeofday(&m_start,NULL);
}

void Timer::stopTimer(){
    gettimeofday(&m_end,NULL);
}

long Timer::getTime(){
    long seconds, useconds;
    seconds  = m_end.tv_sec  - m_start.tv_sec;
    useconds = m_end.tv_usec - m_start.tv_usec;

    if (seconds >= 1 && seconds < 60){
        std::cout << "Time required for execution: " << seconds << " sec " << std::endl;


    }
    else if (seconds >= 60 && seconds < 3600) //minuts
    {
        double min = floor(seconds/60);
        double sec = seconds - min*60 ;
        std::cout << "Time required for execution: " << min << " min"  << std::endl;

    }
    else if(seconds < 1) //milisec
    {
        long time_miliseconds = ((seconds) * 1000 + useconds/1000.0);
        std::cout << "Time required for execution: " << time_miliseconds << " mlsec" << std::endl;

    }
    else //hours
    {
        double hours = floor(seconds/3600);
        double min = floor( (seconds - hours*3600)/60 );
        double sec = seconds - min*60 - hours*3600;
        std::cout << "Time required for execution: " << "hours: " << hours << "min: " << min << "sec: " << sec << std::endl;

    }
    return seconds;
}


}


