#ifndef DRONE_IOEXCEPTION_HPP
#define DRONE_IOEXCEPTION_HPP
#include <string>
#include <exception>
#include <stdexcept>

using namespace std;
namespace drone{

class IOException:runtime_error
{

private:
    const char * m_message;
public:
    IOException(const char * message):runtime_error(message){
        this->m_message=message;
    }
    const char* what() const throw()
       {
           return m_message;
       }

};


}
#endif // DRONE_IOEXCEPTION_HPP
