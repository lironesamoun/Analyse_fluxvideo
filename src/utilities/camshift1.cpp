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
bool color_define=false;
int centerHue;
int centerSat;
int centerVal;
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

    bool save=false;

    Rect trackWindow;

    /*  int hbins = 30, sbins = 32; //number of bin
    int histSize[] = {hbins, sbins};
    float hranges[] = { 0, 180 };//hue range
    float sranges[] = { 0, 256 };//Saturation range
    const float* ranges[] = { hranges, sranges };//hue and saturation range
    MatND hist= Mat::zeros(200,320,CV_8UC3);//Histogramme
    int ch[] = { 0, 1 }; // Index for hue and saturation channel*/
    int hsize = 32; //Number of bin
    float hranges[] = {0,180};//Range value.  varies from 0 to 179
    const float* phranges = hranges;
    int ch[]={0}; //Index for hue channel


    string path="/home/sl001093/Documents/MAM5/PFE/videos/Merio/merio2.avi";
    string outputPath="/home/sl001093/Documents/MAM5/PFE/videos/Merio/save.mp4";
    VideoCapture cap(path);
    int fpsOriginal=cap.get(CV_CAP_PROP_FPS);

    Mat frameRecord;

    cap.read(frameRecord);

    VideoWriter video;
    if (save){
        Size frameSize(static_cast<int>(frameRecord.cols), static_cast<int>(frameRecord.rows));
        video =VideoWriter (outputPath, CV_FOURCC('X','V','I','D') ,fpsOriginal, frameSize,true);

    }


    if( !cap.isOpened() )
    {


        cout << "***Could not initialize video...***\n";

        return -1;
    }

    namedWindow( "Histogram");
    namedWindow( "frame");
    setMouseCallback( "frame", onMouse);



    Mat frame, hsvRoi,hsvImage,trackSelectedRegionHsv, trackSelectedRegionHue, hue, mask, backproj,hist;
    Mat imageDraw;
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
            cvtColor(image, hsvImage, COLOR_BGR2HSV);


            if( trackObject )
            {
                int lw=100;
                int lh=100;
                imageDraw=image.clone();
                /// Compute value of inRange
                rectangle(imageDraw,selection,Scalar(255,255,255),2);

                //Define a ROI around the selected region. It's a roi where we gonna backproject
                cv::Rect rectTrackRegion;
                int centerX=(selection.x)+selection.width/2;
                int centerY=(selection.y)+selection.height/2;
                cv::circle(imageDraw,Point(centerX,centerY),3,Scalar(255,255,255));
                rectTrackRegion.x=centerX-lw;
                rectTrackRegion.y=centerY-lh;

                rectTrackRegion.width=selection.width/2 + lw + selection.width;
                rectTrackRegion.height=selection.height/2 +lh + selection.height;

                rectangle(imageDraw,rectTrackRegion,Scalar(255,0,0),2);

                Mat roiSelected(image, selection),trackSelectedWindow(image,rectTrackRegion);
                namedWindow("roi");
                imshow("roi",roiSelected);
                namedWindow("trackRegion");
                imshow("trackRegion",trackSelectedWindow);
                cvtColor(trackSelectedWindow, trackSelectedRegionHsv, CV_BGR2HSV); //HSV convertion for region around selected region
                vector<Mat> channels1;
                split(trackSelectedRegionHsv, channels1);
                trackSelectedRegionHue = channels1[0];
                // int ch[] = {0, 0};
                //hueTrackSelected.create(trackSelectedRegionHsv.size(), trackSelectedRegionHsv.depth());
                //mixChannels(&trackSelectedRegionHsv, 1, &hueTrackSelected, 1, ch, 1);


                cvtColor(roiSelected, hsvRoi, CV_BGR2HSV); //HSV convertion for selected region
                namedWindow("roiHsv");
                imshow("roiHsv",hsvRoi);


                if (color_define==false){
                    //Get specific hue,sat, value for the selected roi
                    vector<Mat> channels;
                    split(hsvRoi, channels);
                    Mat hueRoi = channels[0];
                    Mat satRoi = channels[1];
                    Mat valRoi = channels[2];


                    centerHue=(int)hueRoi.at<uchar>(hueRoi.rows/2,hueRoi.cols/2);
                    centerSat=(int)satRoi.at<uchar>(satRoi.rows/2,satRoi.cols/2);
                    centerVal=(int)valRoi.at<uchar>(valRoi.rows/2,valRoi.cols/2);
                    /*
                  std::cout << "region center hue: " << (centerHue) << std::endl;
                   std::cout << "region center sat: " << (centerSat) << std::endl;
                    std::cout << "region center val: " << (centerVal) << std::endl;
*/
                    color_define=true;

                }
                inRange(trackSelectedRegionHsv, Scalar(centerHue-30, centerSat-30,centerVal-30),
                        Scalar(centerHue+30, centerSat+30,centerVal+30), mask);


                if( trackObject < 0 )
                {
                    Debug::info("Track object okey");
                    Mat maskroi=mask.clone();

                    calcHist(&trackSelectedRegionHue, 1,ch,maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, CV_MINMAX);


                    trackWindow = selection;
                    trackObject = 1;


                }

                /*
                 namedWindow("hueRoi");
                 imshow("hueRoi",hueRoi);
                 namedWindow("satRoi");
                 imshow("satRoi",satRoi);
                 namedWindow("valRoi");
                 imshow("valRoi",valRoi);*/

                namedWindow("mask");
                imshow("mask",mask);
                calcBackProject(&trackSelectedRegionHue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;
                Debug::info("Go camshift");
                RotatedRect trackBox = CamShift(backproj, trackWindow,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
                namedWindow("camshift");
                imshow("camshift",backproj);
                Debug::info("Camshift finish");
                if( trackWindow.area() <= 1 )
                {
                    Debug::info("Objet perdu");
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                            Rect(0, 0, cols, rows);

                    rectangle(imageDraw,trackWindow,Scalar(0,0,255),5);

                    color_define==true;
                    break;

                }
                ellipse(trackSelectedWindow, trackBox, Scalar(0,0,255), 3, CV_AA );
                namedWindow("frameDraw");
                imshow("frameDraw",imageDraw);
            }
            else if( trackObject < 0 )
                paused = false;


            if( selectObject && selection.width > 0 && selection.height > 0 )
            {
                Mat roi(image, selection);
                bitwise_not(roi, roi);
            }
            /*
            if (imageDraw.empty()){
                 std::cout << "okey" << std::endl;
                imageDraw=image.clone();
            }
            namedWindow("frameDraw");
            imshow("frameDraw",imageDraw);*/
        }



        if (save){
            video << image;
        }





        imshow("frame",image);
        //  waitKey(30);
        waitKey(0);
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
        case 'd':
            trackObject=0;
            break;

        case 'p':
            paused = !paused;
            break;
        default:
            ;
        }

    }

    if (save){
        video.release();
    }

    return 0;
}


