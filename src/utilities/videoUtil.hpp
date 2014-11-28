#ifndef DRONE_VIDEO_UTIL_HPP
#define DRONE_VIDEO_UTIL_HPP

#include "drone.hpp"

using namespace cv;
using namespace std;

namespace drone
{

class VideoUtil
{
public:
  double fps;
  cv::Mat frame;

public:

  static cv::Mat hidePartsVideo(cv::Mat& frame);
  static int run(std::string& path);

public:
    double getFps() const;


};
}//drone


#endif //DRONE_VIDEO_READ_HPP
