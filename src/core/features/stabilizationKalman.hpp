#ifndef DRONE_STABILIZATIONKALMAN_HPP
#define DRONE_STABILIZATIONKALMAN_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

class StabilizationKalman
{
public:
    StabilizationKalman(string &path,string& outputP,bool save=true);

    cv::Mat computeMask(cv::Mat& frame, int lh, int lw);
    void init();

public:
    string path;
    double outputFps;
    string outputPath;
    bool save;

private:
    void init_kalman(double x, double y);
    Point2f kalman_predict_correct(double x, double y);

public:
    static const int HORIZONTAL_BORDER_CROP = 30; // In pixels. Crops the border once the video is stabilized.





};
}//drone


#endif //DRONE_STABILIZATIONKALMAN_HPP
