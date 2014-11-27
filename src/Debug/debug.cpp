#include "../src/Debug/debug.hpp"


namespace drone {

void Debug::error(const std::string &  msg){
    if (DEBUG_LEVEL >= DEBUG_ERROR){
        logE("Error: " + msg);
    }
}
void Debug::warning(const std::string &  msg){
    if (DEBUG_LEVEL >= DEBUG_WARNING){
        logW("Warning: " + msg);
    }
}
void Debug::info(const std::string &  msg){
    if (DEBUG_LEVEL >= DEBUG_INFO){
        logD("Info: " + msg);
    }
}
void Debug::debug(const std::string &  msg){
    if (DEBUG_LEVEL >= DEBUG_DEBUG){
        logD("Debug: " + msg);
    }
}
void Debug::trace(const std::string &  msg){
    if (DEBUG_LEVEL >= DEBUG_TRACE){
        logD("Trace: " + msg);
    }
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

void assert_failure(const char* file, const int line, const char* function, const char* what, const char* cond)
{
        std::cerr << "[" << file << ":" << line << ":" << function << "] "
                  << "ASSERT FAILURE: "	<< what << " [" << cond << "]" << "\n";
        abort();

}

void ensure_error(const char* file, const int line, const char* function, const char* what, int code)
{
    std::string mesg = "(ensure error) " + to_string(what);

}

void fatal_error(const char* file, const int line, const char* function, const char* what, int code)
{

        std::cerr << "[" << file << ":" << line << ":" << function << "] "
                  << "ERROR: "	<< what << "\n";
        exit(code);

}

void ssm_warning(const std::string& what)
{
    std::string msg = "WARNING: " + what;
    std::cout << msg << "\n";

}

void ssm_debug(const int level, const std::string& what)
{

            std::cout << "(" << ((int)(level)) << ") " << what << "\n";


}

void ssm_error(const std::string& what)
{
    fatal_error(__FILE__, __LINE__, __FUNCTION__, what.c_str());
}

}

