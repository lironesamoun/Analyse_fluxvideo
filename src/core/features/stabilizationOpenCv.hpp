#ifndef DRONE_STABILIZATIONCV_HPP
#define DRONE_STABILIZATIONCV_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

class StabilizationOpenCv
{
public:

    StabilizationOpenCv(string& path,string& outputPath, bool save=true,bool stdev=false,
                        bool isTwoPass=false, bool motionFilter=false, bool est_trime=false);
    void run();
    void init();


public:
    double outputFps;
    string path;
    string outputPath;
    Ptr<IFrameSource> stabilizedFrames;
public:
    bool stdev;
    bool save;
    bool isTwoPass;
    bool motionFilter;
    bool est_trime;

};
}//drone


#endif //DRONE_STABILIZATIONCV_HPP
