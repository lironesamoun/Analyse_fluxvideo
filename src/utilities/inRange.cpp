#include "utilities/inRange.hpp"

namespace drone
{

using namespace cv;
using namespace std;

cv::Mat InRange::foundHSVcolor(cv::Mat& frame){

    int rh = 255, rl = 100, gh = 255, gl = 0, bh = 70, bl = 0;
    int H_MIN = 0; // minimum Hue
    int H_MAX = 180; // maximum Hue
    int S_MIN = 0; // minimum Saturation
    int S_MAX = 255; // maximum Saturation
    int V_MIN = 0; // minimum Value
    int V_MAX = 255; //maximum Value

    string windowName = "background";
    namedWindow(windowName,0);


        createTrackbar( "H_MIN", windowName, &H_MIN, H_MAX );
        createTrackbar( "H_MAX", windowName, &H_MAX, H_MAX);
        createTrackbar( "S_MIN", windowName, &S_MIN, S_MAX);
        createTrackbar( "S_MAX", windowName, &S_MAX, S_MAX);
        createTrackbar( "V_MIN", windowName, &V_MIN, V_MAX);
        createTrackbar( "V_MAX", windowName, &V_MAX, V_MAX);


    // for dilation
    Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));

    Mat bgIsolation,hsv;
    int key = 0;
    do
    {
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),bgIsolation);

        //bitwise_not(bgIsolation, bgIsolation);

        erode(bgIsolation, bgIsolation, Mat());
        dilate(bgIsolation, bgIsolation, element);

        imshow(windowName, bgIsolation);
        key = waitKey(33);
    } while((char)key != 27);

    waitKey();


}

}
