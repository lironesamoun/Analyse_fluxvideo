#include "drone.hpp"

using namespace drone;

int main(int argc, char *argv[])
{

    Debug::trace("Titit trace");
       Timer timer;
       timer.startTimer();
       string path="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
       VideoCapture cap(path); // open the video file for reading
       Mat temp,temp1;
       int stepFrame=5;
       namedWindow("frame",1);
       for(;;)
       {
           Mat frame,frame1;
           int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
           std::cout << nbreCurrentFrame << std::endl;

             cap>>frame;
           if (nbreCurrentFrame%(stepFrame)==0){


               temp = frame.clone();

           }
           if (nbreCurrentFrame%(stepFrame+3)==0){


               temp1 = frame.clone();

           }
        imshow("frame",temp);
        namedWindow("frame2",2);
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
       timer.stopTimer();
       timer.getTime();
       return 0;


}

