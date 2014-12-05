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


    // fps counter begin
    time_t start, end;
    int counter = 0;
    double sec;
    double fps;
    bool stepFrame=false;
    // fps counter end

    Timer timerMain;
    timerMain.startTimer();
    string outputPath="/home/sl001093/Documents/MAM5/PFE/videos/videoStabOpenCVResult/test.avi";
    string path1="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
    StabilizationOpenCv stabopenCv(path1,outputPath);
   // stabopenCv.init();
    StabilizationSimple stabSimple(path1);
    // stabSimple.init();
    StabilizationLive stabLive(path1);
     // stabLive.init();
    StabilizationTestSimple stabTest(path1);
    // stabTest.init();
    StabilizationTestSimple2 stabTest2(path1);
    stabTest2.init();



}




    /*
    bool skipFrame=false;


    string path="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
    VideoCapture cap(path); // open the video file for reading
    Mat temp,temp1;
    int stepFrame=10;

    for(;;)
    {

        // fps counter begin
        if (counter == 0){
            time(&start);
        }
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

            namedWindow("frame");
            imshow("frame",temp);
        }
        else {
           //frame=VideoUtil::hidePartsVideo(frame);
            frame=VideoUtil::geometricalCrop(frame,70,0);
            namedWindow("frame");
            imshow("frame",frame);
        }

        // fps counter begin
        time(&end);
        counter++;
        sec = difftime(end, start);
        fps = counter/sec;
        if (counter > 30)
            cout << "Fps: " << fps << endl;

        // overflow protection
        if (counter == (INT_MAX - 1000))
            counter = 0;
        // fps counter end

        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    timerMain.stopTimer();
    timerMain.getTime();
    return 0;


}*/
