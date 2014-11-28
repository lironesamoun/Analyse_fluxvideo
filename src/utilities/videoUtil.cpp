#include "utilities/videoUtil.hpp"

namespace drone
{


/**
  Hide above and below part of a video by putting pixel at black color
  **/
cv::Mat VideoUtil::hidePartsVideo(cv::Mat& frame){
     int cols=frame.cols;
     int rows=frame.rows;
     Debug::info("Size frame: " + to_string(cols) + "*" + to_string(rows));
     for(int i = 1;i<rows; i++){
                     for(int j=1; j<cols; j++){
                        // Vec3b color = temp.at<Vec3b>(Point(j,i));
                         if (i < rows/7 || i > (rows-rows/7)){
                             frame.at<Vec3b>(i,j)=Vec3b(0,0,0);

                         }

         }
             }
     return frame;
}



/**
  Only read the video
  **/
int VideoUtil::run(std::string& path){

    VideoCapture cap(path); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    int fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    Debug::info("Frame per seconds : " + to_string(fps));


    namedWindow("Video",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while(1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video
        cap >> frame;
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

double VideoUtil::getFps() const{
    return this->fps;
}

}
