#ifndef DRONE_ISTABILIZATION_HPP
#define DRONE_ISTABILIZATION_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"
#include "core/features/stabilizationSimple.hpp"

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
      static const int ISTABILIZATION_PREPROCESSING = 2;

      static const int ISTABILIZATION_BASICSURF = 3;
      static const int IBOWFEATURE_BASICGOODFEATURE = 4;

public:
    void run(string& path,int method);




};
}//drone


#endif //DRONE_ISTABILIZATION_HPP
