#include "drone.hpp"

using namespace drone;

cv::Mat skipNFrames(VideoCapture& cap,cv::Mat& frame, int n)
{
    cv::Mat temp;
    int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
    Debug::info("Current frame: " + nbreCurrentFrame);
    if (nbreCurrentFrame%(n)==0){
        temp=frame.clone();
    }

    return temp;
}

int main(int argc, char *argv[])
{

    Timer timerMain;
    timerMain.startTimer();
    string outputPath="/home/sl001093/Documents/MAM5/PFE/videos/videoStabOpenCVResult/test.avi";
    string path1="/home/sl001093/Documents/MAM5/PFE/videos/morceau5.avi";
    StabilizationOpenCv stabopenCv(path1,outputPath);
    stabopenCv.init();
/*
    bool skipFrame=false;


    string path="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
    VideoCapture cap(path); // open the video file for reading
    Mat temp,temp1;
    int stepFrame=10;

    for(;;)
    {
        Mat frame;
        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
        Debug::trace("Current frame: " + to_string(nbreCurrentFrame));


        cap>>frame;
        if (skipFrame){
            if (nbreCurrentFrame%(stepFrame)==0){

                temp = frame.clone();

            }
            if (nbreCurrentFrame%(stepFrame+3)==0){


                temp1 = frame.clone();

            }

            /*namedWindow("frame");
            imshow("frame",temp);
            namedWindow("frame2");
            imshow("frame2",temp1);
            cv::Mat difference=temp-temp1;
            namedWindow("difference",3);
            imshow("difference",difference);
            temp=VideoUtil::hidePartsVideo(temp);

            namedWindow("frame");
            imshow("frame",temp);
        }
        else {
           frame=VideoUtil::hidePartsVideo(frame);
            namedWindow("frame");
            imshow("frame",frame);
        }


        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }
    */
    timerMain.stopTimer();
    timerMain.getTime();
    return 0;


}

