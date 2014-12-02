#ifndef DRONE_STABILIZATION_HPP
#define DRONE_STABILIZATION_HPP

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

    StabilizationOpenCv(string& path,string& outputPath,bool stdev=false, bool save_motion=true,
                        bool isTwoPass=false, bool motionFilter=false, bool est_trime=false);
    void run();
    void saveMotionsIfNecessary();
    void init();


public:
    double outputFps;
    string path;
    string outputPath;
    Ptr<IFrameSource> stabilizedFrames;
public:
    bool stdev;
    bool save_motion;
    bool isTwoPass;
    bool motionFilter;
    bool est_trime;

};
}//drone


#endif //DRONE_STABILIZATION_HPP
