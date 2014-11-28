#include "drone.hpp"

using namespace drone;


int main(int argc, char *argv[])
{

    bool skipFrame=true;

    Timer timerMain;
    timerMain.startTimer();
    string path="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
    VideoCapture cap(path); // open the video file for reading
    Mat temp,temp1;
    int stepFrame=5;


    for(;;)
    {
        Mat frame,frame1;
        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
        Debug::trace("Current frame: " + to_string(nbreCurrentFrame));


        cap>>frame;
        if (nbreCurrentFrame%(stepFrame)==0){

            temp = frame.clone();

        }
        if (nbreCurrentFrame%(stepFrame+3)==0){


            temp1 = frame.clone();

        }
        namedWindow("frame");
        imshow("frame",temp);
        namedWindow("frame2");
        imshow("frame2",temp1);
        cv::Mat difference=temp-temp1;
        namedWindow("difference",3);
        imshow("difference",difference);

        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }
    timerMain.stopTimer();
    timerMain.getTime();
    return 0;


}

