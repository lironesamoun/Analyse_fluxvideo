#ifndef DRONE_CAMSHIFTPERSO_HPP
#define DRONE_CAMSHIFTPERSO_HPP

#include "drone.hpp"

using namespace cv;
using namespace std;

namespace drone
{

class CamshiftPerso
{
public:
    CamshiftPerso(bool backprojMode=false,bool selectObject=false,int trackObject=0);

    cv::Mat run(cv::Mat& frame);

    void activeBackProjMode();//Display backproj
    void selectObjectAutomatic(); // Select an object automatically





public:
    Mat image;//Frame

    bool backprojMode;

    int trackObject; // Status of tracked object
    bool showHist; // show histogramme
    Point origin;// Origin point
    Rect selection;//Selection of the ROI
     bool selectObject;

    int vmin;
    int vmax;
    int smin;


public:


};
}//drone


#endif //DRONE_CAMSHIFT_HPP
