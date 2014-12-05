#ifndef DRONE_STABILIZATIONLIVE_HPP
#define DRONE_STABILIZATIONLIVE_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{
//modified by chen jia.
//email:chenjia2013@foxmail.com
class StabilizationLive
{
public:
    StabilizationLive(string &path);

    void init();

public:
   static  const int SMOOTHING_RADIUS = 30; // In frames. The larger the more stable the video, but less reactive to sudden panning
   static const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.


public:
    string path;



};
}//drone


#endif //DRONE_STABILIZATIONLIVE_HPP
