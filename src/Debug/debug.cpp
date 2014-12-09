#include "../src/Debug/debug.hpp"


namespace drone {



void Debug::error(const std::string &  msg){
        logE("Error: " + msg);

}
void Debug::warning(const std::string &  msg){
        logW("Warning: " + msg);

}
void Debug::info(const std::string &  msg){
        logD("Info: " + msg);

}
void Debug::debug(const std::string &  msg){
        logD("Debug: " + msg);

}
void Debug::trace(const std::string &  msg){
        logD("Trace: " + msg);

}


void Debug::logE(const std::string & msg){
        std::cerr << msg << std::endl;
}
void Debug::logD(const std::string & msg){
        std::cout << msg << std::endl;
}
void Debug::logW(const std::string & msg){
        std::cout << msg << std::endl;

}

void fatal_error(const char* file, const int line, const char* function, const char* what, int code)
{
        std::cerr << "[" << file << ":" << line << ":" << function << "] "
                  << "ERROR: "	<< what << "\n";
        exit(code);

}


void drone_warning(const std::string& what)
{
    std::string msg = "WARNING: " + what;
    std::cout << msg << "\n";

}



void drone_error(const std::string& what)
{
    fatal_error(__FILE__, __LINE__, __FUNCTION__, what.c_str());
}


void ensure_error(const char* file, const int line, const char* function, const char* what, int code)
{
    std::string mesg = "(ensure error) " + to_string(what);
    drone::fatal_error(file,line,function,mesg.c_str(),code);
}

}

