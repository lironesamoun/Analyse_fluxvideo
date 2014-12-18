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

#define drone_ensure(cond,what) if (!(cond)) drone::ensure_error(__FILE__, __LINE__, __FUNCTION__, what);

namespace drone {

void ensure_error(const char* file, const int line, const char* function, const char* what, int code=-1);
void fatal_error(const char* file, const int line, const char* function,const char* what, int code=-1);
void imageretrieval_error(const std::string& what);
void imageretrieval_warning(const std::string& what);

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
