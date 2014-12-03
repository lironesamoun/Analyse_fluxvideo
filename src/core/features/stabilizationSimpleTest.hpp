#ifndef DRONE_STABILIZATIONTESTSIMPLE_HPP
#define DRONE_STABILIZATIONTESTSIMPLE_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

class StabilizationTestSimple
{
public:
    StabilizationTestSimple(string &path);

    void init();


public:
    string path;


};
}//drone


#endif //DRONE_STABILIZATIONSIMPLE_HPP
