#include "utilities/videoRead.hpp"

namespace drone
{

cv::Mat VideoRead::skipNFrames(VideoCapture capture, int n)
{
    cv::Mat frame;
    cv::Mat temp;

              return temp; capture.set(CV_CAP_PROP_POS_MSEC,3000);
              capture>>frame;
              std::cout << "ok" << std::endl;
              temp = frame.clone();





}

/**
  Only read the video
  **/
int VideoRead::run(std::string& path){

    VideoCapture cap(path); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    this->fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

     cout << "Frame per seconds : " << fps << endl;

    namedWindow("Video",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while(1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video
        cap >> this->frame;
         if (!bSuccess) //if not success, break loop
        {
                        cout << "Cannot read the frame from video file" << endl;
                       break;
        }

        imshow("Video", frame); //show the frame in "Video" window

        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
       {
                cout << "esc key is pressed by user" << endl;
                break;
       }
    }

    return 0;
}

double VideoRead::getFps() const{
    return this->fps;
}

}
