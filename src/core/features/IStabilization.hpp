#ifndef DRONE_ISTABILIZATION_HPP
#define DRONE_ISTABILIZATION_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"


using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

class IStabilization
{
public:

      IStabilization();

public:

      static const int ISTABILIZATION_OPENCV = 0;
      static const int ISTABILIZATION_KALMAN = 1;

public:
    void run(string& path,string& outPath,int method,bool save);




};
}//drone


#endif //DRONE_ISTABILIZATION_HPP
