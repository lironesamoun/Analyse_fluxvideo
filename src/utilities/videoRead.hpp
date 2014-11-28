#ifndef DRONE_VIDEO_READ_HPP
#define DRONE_VIDEO_READ_HPP

#include "drone.hpp"

using namespace cv;
using namespace std;

namespace drone
{

class VideoRead
{
public:
  double fps;
  cv::Mat frame;

public:

  cv::Mat skipNFrames(VideoCapture &cap, Mat &frame, int n);
   int run(std::string& path);

public:
    double getFps() const;


};
}//drone


#endif //DRONE_VIDEO_READ_HPP
