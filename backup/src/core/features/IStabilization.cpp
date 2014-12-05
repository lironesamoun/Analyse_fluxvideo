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

void IStabilization::run(string& path,int method){
    if (method==ISTABILIZATION_PREPROCESSING){
        StabilizationSimple stabSimple;
        stabSimple.run(path);
    }
}

}






