#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "drone.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <ctype.h>

using namespace cv;
using namespace std;

using namespace drone;

Mat image;

bool backprojMode = true;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int H_MIN = 0; // minimum Hue
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
        Debug::info("Selection width: " + to_string(selection.width) + "\n Selection height: " + to_string(selection.height)  );
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

    int hbins = 30, sbins = 32; //number of bin
    int histSize[] = {hbins, sbins};
    float hranges[] = { 0, 180 };//hue range
    float sranges[] = { 0, 256 };//Saturation range
    const float* ranges[] = { hranges, sranges };//hue and saturation range
    MatND hist= Mat::zeros(200,320,CV_8UC3);//Histogramme
    int ch[] = { 0, 1 }; // Index for hue and saturation channel

    string path="/home/sl001093/Documents/MAM5/PFE/videos/aerial3.mp4";
    VideoCapture cap(path);

    if( !cap.isOpened() )
    {


        cout << "***Could not initialize video...***\n";

        return -1;
    }

    namedWindow( "Histogram");
    namedWindow( "frame");
    setMouseCallback( "frame", onMouse);



    Mat frame, hsv, hue, mask, backproj;
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
            //Noise reduction of windows
            Size kSize;
            kSize.height = 5;
            kSize.width = 5;
            double sigma = 0.3*(3/2 - 1) + 0.8;
            GaussianBlur(image,image,kSize,sigma,0.0,4);
            cvtColor(image, hsv, COLOR_BGR2HSV);


            if( trackObject )
            {
                int lw=100;
                int lh=100;

                /// Compute value of inRange
                rectangle(image,selection,Scalar(255,255,255),2);

                //Define a ROI around the selected region
                cv::Rect rectTrackRegion;
                int centerX=(selection.x)+selection.width/2;
                int centerY=(selection.y)+selection.height/2;
                rectTrackRegion.x=centerX-lw;
                rectTrackRegion.y=centerY-lh;
                rectTrackRegion.width=selection.width*2+lw;
                rectTrackRegion.height=selection.height*2+lh;

                 rectangle(image,rectTrackRegion,Scalar(255,0,0),2);

                Mat roiSelected(image, selection);
                namedWindow("roi");
                imshow("roi",roiSelected);

                cvtColor(roiSelected, hsv, CV_BGR2HSV); //HSV convertion

                //int _vmin = vmin, _vmax = vmax;


                // inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                //       Scalar(180, 256, MAX(_vmin, _vmax)), mask);
            }
        }

        imshow("frame",image);
        waitKey(0);
        char c = (char)waitKey(10);
        if( c == 27 )
            break;

    }

    return 0;
}


