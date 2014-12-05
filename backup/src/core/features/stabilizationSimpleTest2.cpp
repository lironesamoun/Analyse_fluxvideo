#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "core/features/stabilizationSimpleTest2.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;

#define MAX_COUNT 250
#define DELAY_T 3
#define PI 3.1415

namespace drone
{

StabilizationTestSimple2::StabilizationTestSimple2(string &path){
    this->path=path;
    // For further analysis


}

void StabilizationTestSimple2::init(){
    time_t start, end;
    int counter = 0;
    double sec;
    double fps;
    int compteur=0;
    //Read the video
    VideoCapture cap(path);
    Mat currImg, colorImg, outImg, grayImg;


    cap.read(colorImg);
    VideoUtil::geometricalCrop(colorImg,70,0);//Crop the picture
    cvtColor(colorImg,grayImg,CV_BGR2GRAY);
    currImg = grayImg.clone();// Current picture



    Mat refFrame;
    cap.read(refFrame);//Frame +1
    VideoUtil::geometricalCrop(refFrame,70,0);
    cvtColor(refFrame,refFrame,CV_BGR2GRAY); // Frame +1


    namedWindow("Stabilize");
    namedWindow("GoodMatches");
    Mat temp;
    Mat currentFrame=refFrame;


    for (;;){
    Mat matrixTransform=Mat::eye(Size(3,3),CV_32FC1);
        // fps counter begin
        if (counter == 0){
            time(&start);
        }
        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
        //  cap.read(colorImg);

        VideoUtil::geometricalCrop(colorImg,70,0);// Crop the video
        Debug::trace("Current frame: " + to_string(nbreCurrentFrame));
        currentFrame.copyTo(refFrame);//Get the reference frame


        cap.read(colorImg);
        VideoUtil::geometricalCrop(colorImg,70,0);
        cvtColor(colorImg,grayImg,CV_BGR2GRAY);
        currentFrame =  grayImg.clone();//Get the current frame




        vector<Point2f> cornersPrevious;//Stock features of reference Frame
        cornersPrevious.reserve(400);

        vector<Point2f> cornersCurr;//Stock features of current frame
        cornersCurr.reserve(400);

        TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);

        goodFeaturesToTrack(refFrame,cornersPrevious,400,0.01,5.0);

        cornerSubPix(refFrame,cornersPrevious,Size(20,20),Size(-1,-1),opticalFlowTermCriteria);
        // Debug::trace("Size of feature track : " + to_string(cornersPrevious.size()));


        vector<uchar> featureFound; // status of tracked features
        featureFound.reserve(400);

        vector<float> featureErrors; // error in tracking
        featureErrors.reserve(400);




        calcOpticalFlowPyrLK(refFrame,currentFrame,cornersPrevious,cornersCurr,featureFound,featureErrors,Size(20,20),3,
                             cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3),0,0.0001);




        // keep the good points
        std::vector<cv::Point2f> initialPoints;
        std::vector<cv::Point2f> trackedPoints;
        for (int i=0;i<cornersCurr.size();i++){
            double motion = sqrt(pow(cornersPrevious.at(i).x-cornersCurr.at(i).x,2)+pow(cornersPrevious.at(i).y-cornersCurr.at(i).y,2));
            // std::cout << "Motion: " << motion << std::endl;
            if (featureFound[i] && motion < 20 ){
                //Keep this point in vector
                initialPoints.push_back(cornersPrevious.at(i));
                trackedPoints.push_back(cornersCurr.at(i));
            }
        }

        Debug::info("Nbre de features avant: " + to_string(cornersPrevious.size()));
        Debug::info("Nbre de features après: " + to_string(trackedPoints.size()));
        // draw the tracking effect
        cv::Mat drawingPoint;
        currentFrame.copyTo(drawingPoint);

        cvtColor(drawingPoint,drawingPoint,CV_GRAY2RGB);
        // for all tracked points
        for(uint i= 0; i < initialPoints.size(); i++ ) {
            // draw line and circle
            cv::line(drawingPoint,
                     initialPoints[i], // initial position
                     trackedPoints[i], // new position
                     cv::Scalar(255,255,255));
            cv::circle(drawingPoint, trackedPoints[i], 3, cv::Scalar(0,0,255),-1);
        }

        namedWindow("Tracking point");
        imshow("Tracking point",drawingPoint);
        //Compute the homography
        if (initialPoints.size() >4 && trackedPoints.size()>4){

             //cv::Mat transformMatrix = findHomography(initialPoints,trackedPoints,CV_RANSAC,3);

          Mat transformMatrix = estimateGlobalMotionRobust(initialPoints,trackedPoints,3,RansacParams::affine2dMotionStd(),0,0);

            // Mat transformMatrix = estimateGlobalMotionLeastSquares(initialPoints,trackedPoints,AFFINE,0);
            //Mat transformMatrix = estimateRigidTransform(initialPoints,trackedPoints ,false); // false = rigid transform, no scaling/shearing
            // warpAffine(refFrame,outImg,transformMatrix,refFrame.size(), INTER_NEAREST|WARP_INVERSE_MAP, BORDER_CONSTANT);

            std::cout << "type: " << VideoUtil::type2str(transformMatrix.type()) << std::endl;

            matrixTransform=matrixTransform.mul(transformMatrix);

            warpPerspective(currentFrame,outImg,matrixTransform,refFrame.size(), INTER_LINEAR |WARP_INVERSE_MAP,BORDER_CONSTANT ,0);
            namedWindow("Stabilized");

        }
        else {
            Mat transformMatrix = estimateGlobalMotionRobust(cornersPrevious,cornersCurr,3,RansacParams::affine2dMotionStd(),0,0);


            warpPerspective(currentFrame,outImg,transformMatrix,refFrame.size(), INTER_LINEAR |WARP_INVERSE_MAP,BORDER_CONSTANT ,0);
        }


        namedWindow("Stabilized");
        imshow("Stabilized",outImg);


        // fps counter begin
        std::cout << "okey"<< std::endl;
        time(&end);
        counter++;
        sec = difftime(end, start);
        fps = counter/sec;
        // if (counter > 30)
        std::cout << "Fps: " << fps << std::endl;

        // overflow protection
        if (counter == (INT_MAX - 1000))
            counter = 0;
        // fps counter end

        compteur++;
        if(waitKey(27) >= 0) break;

    }
}

}





