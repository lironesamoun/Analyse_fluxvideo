#include "utilities/camshiftPerso.hpp"

namespace drone
{





CamshiftPerso::CamshiftPerso(bool backprojMode,bool selectObject,int trackObject)
{
    this->backprojMode=backprojMode;
    this->selectObject=selectObject;
    this->trackObject=trackObject;
}

cv::Mat CamshiftPerso::run(cv::Mat& frame){

    bool debugMode=false;
    Point origin(20,20);
    int x=170;
    int y=100;
    selection.x = x;
    selection.y = y;
    selection.width =300;
    selection.height = 300;

    selection &= Rect(0, 0, frame.cols, frame.rows);
    rectangle(frame,selection, Scalar(255,255,255),5);

    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;


    Debug::trace("Size selection: " + to_string(selection.size()));
    if (debugMode){
    namedWindow("rectangle");
    imshow("rectangle",frame);
    }
   // waitKey(0);



    Mat  hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;


    frame.copyTo(image);


        cvtColor(image, hsv, COLOR_BGR2HSV);

        Debug::trace("State trackObject " + to_string(trackObject));
        if( trackObject )
        {
            Debug::trace("Okey!");
            int _vmin = vmin, _vmax = vmax;

            inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                    Scalar(180, 256, MAX(_vmin, _vmax)), mask);
            int ch[] = {0, 0};
            hue.create(hsv.size(), hsv.depth());
            mixChannels(&hsv, 1, &hue, 1, ch, 1);


            if (debugMode){
                namedWindow("hsv");
                imshow("hsv",hsv);
            }


            if( trackObject < 0 )
            {
                Mat roi(hue, selection), maskroi(mask, selection);
                //namedWindow("roi");
                //imshow("roi",roi);
                //waitKey(0);
                calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                normalize(hist, hist, 0, 255, CV_MINMAX);

                trackWindow = selection;
                trackObject = 1;
            }


            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
            RotatedRect trackBox = CamShift(backproj, trackWindow,
                                            TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
            if( trackWindow.area() <= 1 )
            {
                int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                   trackWindow.x + r, trackWindow.y + r) &
                        Rect(0, 0, cols, rows);
            }

            if( backprojMode )
                cvtColor( backproj, image, COLOR_GRAY2BGR );
            ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
        }



    if( selectObject && selection.width > 0 && selection.height > 0 )
    {
        Mat roi(image, selection);
        bitwise_not(roi, roi);
    }

    namedWindow("frame");
    imshow("frame",image);



    return image;




}



}
