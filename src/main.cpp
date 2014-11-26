#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include "fstream"
#include "iostream"
#include "utilities/videoRead.hpp"


using namespace cv;
using namespace std;
using namespace drone;

int main(int argc, char *argv[])
{

    VideoRead video;
    video.run("/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi");


}

