#ifndef DRONE_STABILIZATIONSIMPLE_HPP
#define DRONE_STABILIZATIONSIMPLE_HPP

#include "drone.hpp"
#include "opencv2/videostab/stabilizer.hpp"
#include "core/features/IStabilization.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

namespace drone
{

// This video stablisation smooths the global trajectory using a sliding average window

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video
class StabilizationSimple
{
public:
    StabilizationSimple();

    void run(string& path);

public:
   static  const int SMOOTHING_RADIUS = 30; // In frames. The larger the more stable the video, but less reactive to sudden panning
    static const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.


public:
    string path;



};
}//drone


#endif //DRONE_STABILIZATIONSIMPLE_HPP
