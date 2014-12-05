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
    cv::Mat computeMask(cv::Mat& frame, int lh, int lw);
    void init_kalman(double x, double y);
    Point2f kalman_predict_correct(double x, double y);

    void init();


public:
    string path;



};
}//drone


#endif //DRONE_STABILIZATIONSIMPLE2_HPP
