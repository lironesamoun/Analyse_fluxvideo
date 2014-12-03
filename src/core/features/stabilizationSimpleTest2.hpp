#ifndef DRONE_STABILIZATIONTESTSIMPLE2_HPP
#define DRONE_STABILIZATIONTESTSIMPLE2_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

class StabilizationTestSimple2
{
public:
    StabilizationTestSimple2(string &path);

    void init();


public:
    string path;


};
}//drone


#endif //DRONE_STABILIZATIONSIMPLE2_HPP
