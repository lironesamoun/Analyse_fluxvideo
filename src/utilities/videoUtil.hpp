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
  cv::Mat frame;

public:

  static cv::Mat hidePartsVideo(cv::Mat& frame);
  static cv::Mat geometricalCrop(cv::Mat& frame,int lh,int lw);
  static int run(std::string& path);
  static string type2str(int type);
  static cv::Mat computeMask(Mat& frame,int lh,int lw);
  static cv::Mat skipNFrames(VideoCapture& cap,cv::Mat& frame, int n);

public:


};
}//drone


#endif //DRONE_VIDEO_READ_HPP
