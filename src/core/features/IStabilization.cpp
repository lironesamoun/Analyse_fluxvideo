#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "core/features/IStabilization.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;


namespace drone
{

IStabilization::IStabilization(){

}

void IStabilization::run(string& path,string& outPath,int method,bool save){
    drone_ensure(method < 2, "Method of stabilization doesn't exist");
    if (method==IStabilization::ISTABILIZATION_OPENCV){
           StabilizationOpenCv stabopenCv(path,outPath);
           stabopenCv.init();
    }
    else if (method==IStabilization::ISTABILIZATION_KALMAN){
        StabilizationKalman stabKalman(path,outPath,save);
        stabKalman.init();

    }
    else {
        Debug::error("No Method selected");
    }

}

}






