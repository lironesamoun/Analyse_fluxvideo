#ifndef DRONE_INRANGE_HPP
#define DRONE_INRANGE_HPP

#include "drone.hpp"

using namespace cv;
using namespace std;

namespace drone
{

class InRange
{
public:
    static cv::Mat foundHSVcolor(cv::Mat& frame);
public:
     cv::Mat image;


public:


};
}//drone


#endif //DRONE_INRANGE_HPP
