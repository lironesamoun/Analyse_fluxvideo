#ifndef DRONE_VIDEO_READ_HPP
#define DRONE_VIDEO_READ_HPP

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include "fstream"
#include "iostream"

using namespace cv;
using namespace std;

namespace drone
{

class VideoRead
{
public:

public:

    int run(std::string path);

};
}//drone


#endif //DRONE_VIDEO_READ_HPP
