/*#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <ctype.h>

using namespace cv;
using namespace std;


Mat image;

bool backprojMode = true;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 160, vmax = 256, smin =0;
int H_MIN = 25; // minimum Hue
int H_MAX = 180; // maximum Hue
int S_MIN = 0; // minimum Saturation
int S_MAX = 255; // maximum Saturation
int V_MIN = 0; // minimum Value
int V_MAX = 255; //maximum Value
const string trackbarWindowName = "Trackbars";

static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1; // L'objet est selectionné si la hauteur et la largeur du rectangle sont >0
        break;
    }
}
void on_trackbar( int, void* )
{

// Doing nothing here since we are going to handle the changes in some other place

}

void createTrackbars(){

    namedWindow(trackbarWindowName,0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf( TrackbarName, "H_MIN", H_MIN);
    sprintf( TrackbarName, "H_MAX", H_MAX);
    sprintf( TrackbarName, "S_MIN", S_MIN);
    sprintf( TrackbarName, "S_MAX", S_MAX);
    sprintf( TrackbarName, "V_MIN", V_MIN);
    sprintf( TrackbarName, "V_MAX", V_MAX);

    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );

}



int main( int argc, const char** argv )
{



    Rect trackWindow;
    int hsize = 32; //Number of bin
    float hranges[] = {0,180};//Range value.  varies from 0 to 179
    const float* phranges = hranges;
    int ch[]={0}; //Index for hue channel



   // int hbins = 30, sbins = 32;
     //   int histSize[] = {hbins, sbins};
       // float hranges[] = { 0, 180 };
        //float sranges[] = { 0, 256 };
        //const float* ranges[] = { hranges, sranges };
         //MatND hist= Mat::zeros(200,320,CV_8UC3);
         //int ch[] = { 0, 1 };

    string path="/home/sl001093/Documents/MAM5/PFE/videos/aerial3.mp4";
    VideoCapture cap(path);

    if( !cap.isOpened() )
    {


        cout << "***Could not initialize video...***\n";

        return -1;
    }

    namedWindow( "Histogram");
    namedWindow( "CamShift Demo");
    setMouseCallback( "CamShift Demo", onMouse);
     createTrackbars(); // create trackbars





    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    for(;;)
    {

        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                break;
        }

        frame.copyTo(image);

        if( !paused )
        {
            //Noise reduction
            Size kSize;
            kSize.height = 5;
            kSize.width = 5;
            double sigma = 0.3*(3/2 - 1) + 0.8;
            GaussianBlur(image,image,kSize,sigma,0.0,4);
            cvtColor(image, hsv, COLOR_BGR2HSV);


            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;


               // inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                 //       Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                inRange(hsv,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),mask);

                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
                    Mat roi(hue, selection), maskroi(mask, selection);
                    //calcHist(const Mat* images, int nimages, const int* channels, InputArray mask, OutputArray hist, int dims, const int* histSize, const float** ranges,
                     //       bool uniform=true, bool accumulate=false )
                    calcHist(&roi, 1,ch, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, CV_MINMAX);

                    trackWindow = selection;
                    trackObject = 1;

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, CV_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
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
        }
        else if( trackObject < 0 )
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        imshow( "CamShift Demo", image);
        imshow( "Histogram", histimg );
       // waitKey(0);
       //  waitKey(1000);

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
        case 'b':
            backprojMode = !backprojMode;
            break;
        case 'c':
            trackObject = 0;
            histimg = Scalar::all(0);
            break;
        case 'h':
            showHist = !showHist;
            if( !showHist )
                destroyWindow( "Histogram" );
            else
                namedWindow( "Histogram" );
            break;
        case 'p':
            paused = !paused;
            break;
        default:
            ;
        }
    }

    return 0;
}

*/
