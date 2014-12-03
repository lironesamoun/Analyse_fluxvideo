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
  Geometrical crop of a frame from a video
  Delete above and below part of a video
  **/
cv::Mat VideoUtil::geometricalCrop(cv::Mat& frame,int lh,int lw){
    cv::Mat roi;//Final region
    int height=frame.rows;
    int width=frame.cols;
    int heightLimit=height-lh;
    int WidthLimit=width-lw;
    roi=frame(Range(lh,heightLimit),Range(lw,WidthLimit));//(haut en bas)(gauche à droite)
    frame=roi;

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

string VideoUtil::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

}
