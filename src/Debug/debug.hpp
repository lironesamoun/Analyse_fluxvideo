#ifndef DRONE_DEBUG_HPP
#define DRONE_DEBUG_HPP

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>

#define LOGD printf
#define LOGW printf
#define LOGE printf

namespace drone {
void ssm_debug(const int level, const std::string& what);
void ssm_error(const std::string& what);
void ssm_warning(const std::string& what);

/**
  * This function converts any class or data type to a string. In C++11,
  * to_string is finally added to the <a href=http://en.cppreference.com/w/cpp/string/basic_string/to_string
  > standard. </a>
  * @param t Template class
  * @return converted string
  */
template <class T> std::string to_string( const T &t )
{
    std::ostringstream oss;
    oss << t;
    return std::string (oss.str());
}

template <class T> std::string to_string( const std::vector<T> &t )
{
    std::ostringstream oss;
    for(int i=0; i<t.size(); i++)
    {
        oss << t.at(i) << "\t";
    }
    return std::string (oss.str());
}
class Debug{

public:
    static const int DEBUG_OFF=0;
    static const int DEBUG_ERROR=1;
    static const int DEBUG_WARNING=2;
    static const int DEBUG_INFO=3;
    static const int DEBUG_DEBUG=4;
    static const int DEBUG_TRACE=5;
    static const int DEBUG_LOWLEVEL=6;

private:
    static int DEBUG_LEVEL;
public:
    static void enabledErrorLevel(){
        DEBUG_LEVEL=DEBUG_ERROR;
    }
    static void enabledWarningLevel(){
        DEBUG_LEVEL=DEBUG_WARNING;
    }
    static void enabledInfoLevel(){
        DEBUG_LEVEL=DEBUG_INFO;
    }
    static void enabledDebugLevel(){
        DEBUG_LEVEL=DEBUG_DEBUG;
    }
    static void enabledTraceLevel(){
        DEBUG_LEVEL=DEBUG_TRACE;
    }
    static void enabledLowLevel(){
        DEBUG_LEVEL=DEBUG_LOWLEVEL;
    }
    static void disabledDebug(){
        DEBUG_LEVEL=DEBUG_OFF;
    }
    static void setDebugLevel(int debug_level){
        DEBUG_LEVEL=debug_level;
    }
    static int getDebugLevel(){
        return DEBUG_LEVEL;
    }
    static void error(const std::string &  msg);
    static void warning(const std::string &  msg);
    static void info(const std::string &  msg);
    static void debug(const std::string &  msg);
    static void trace(const std::string &  msg);
    static void lowlevel(const std::string &  msg);
private:
    static void logD(const std::string & msg);
    static void logW(const std::string & msg);
    static void logE(const std::string & msg);

};



}

#endif //DRONE_DEBUG_HPP
