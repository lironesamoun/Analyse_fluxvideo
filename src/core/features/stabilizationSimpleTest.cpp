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
#include "core/features/stabilizationSimple.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;

#define MAX_COUNT 250
#define DELAY_T 3
#define PI 3.1415

namespace drone
{

StabilizationTestSimple::StabilizationTestSimple(string &path){
    this->path=path;
    // For further analysis


}

void StabilizationTestSimple::init(){
    //Read the video
    //Read the video
    VideoCapture cap(path);
    Mat currImg, colorImg, outImg, grayImg, backupColorImg;


    Mat refFrame;
    cap.read(refFrame);//Frame +1
   // VideoUtil::geometricalCrop(refFrame,70,0);
    cvtColor(refFrame,refFrame,CV_BGR2GRAY); // Frame +1
    SurfDescriptorExtractor extractor;

    int minHessian = 200;

    SurfFeatureDetector detector(minHessian);


    namedWindow("Stabilize");

    Mat temp;
    Mat currentFrame=refFrame;
    for (;;){
        int nbr_Frame=cap.get(CV_CAP_PROP_FRAME_COUNT);
          Debug::trace("Number total frame: " + to_string(nbr_Frame));
        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
//        cap.read(colorImg);

       // VideoUtil::geometricalCrop(colorImg,70,0);// Crop the video
        Debug::trace("Current frame: " + to_string(nbreCurrentFrame));
        currentFrame.copyTo(refFrame);//Get the reference frame


        cap.read(colorImg);
        //VideoUtil::geometricalCrop(colorImg,70,0);
        cvtColor(colorImg,grayImg,CV_BGR2GRAY);
        currentFrame =  grayImg.clone();//Get the current frame

       /* namedWindow("Prev");
        imshow("Prev",refFrame);
        namedWindow("current");
        imshow("current",currentFrame);
        waitKey(3000);*/


        vector<KeyPoint> keyPointRef;
        vector<KeyPoint> keyPointCurr;
        Mat descriptorRef;
        Mat descriptorCurr;

        detector.detect(refFrame,keyPointRef);
        extractor.compute(refFrame,keyPointRef,descriptorRef);

       currImg= VideoUtil::geometricalCrop(currentFrame,70,0);
        detector.detect(currImg,keyPointCurr);
        extractor.compute(currImg,keyPointCurr,descriptorCurr);

        FlannBasedMatcher matcher;
        vector<DMatch> matches;
        matcher.match(descriptorCurr,descriptorRef,matches);

        double maxDist = 0, minDist = 100;
        for(int i = 0; i < descriptorCurr.rows; i++)
        {
            double dist = matches[i].distance;

            if(dist < minDist) minDist = dist;
            if(dist > maxDist) maxDist = dist;
        }

        vector<DMatch>good_matches;
        for (int i = 0; i < descriptorCurr.rows; i++)
        {

            if(matches[i].distance < 0.1)
            {
                good_matches.push_back(matches[i]);
            }
        }

        vector<Point2f>curPoint;
        vector<Point2f>refPoint;

        for (int i = 0; i < good_matches.size(); i++)
        {
            curPoint.push_back(keyPointCurr[good_matches[i].queryIdx].pt);
            refPoint.push_back(keyPointRef[good_matches[i].trainIdx].pt);
        }

        Mat imgMatches;
      //  drawMatches(currentFrame,keyPointCurr,refFrame,keyPointRef,good_matches,imgMatches,Scalar::all(-1),Scalar::all(-1),vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // namedWindow("GoodMatches",0);
        //imshow("GoodMatches",imgMatches);




       Mat transformMatrix = estimateGlobalMotionRobust(refPoint,curPoint,3);

       // Mat transformMatrix = estimateRigidTransform(refPoint,curPoint ,false); // false = rigid transform, no scaling/shearing


       // warpAffine(refFrame,outImg,transformMatrix,refFrame.size(), INTER_NEAREST|WARP_INVERSE_MAP, BORDER_CONSTANT);
          warpPerspective(currentFrame,outImg,transformMatrix,refFrame.size(), INTER_LINEAR |WARP_INVERSE_MAP,BORDER_CONSTANT ,0);





      //  imshow("Input",colorImg);

        imshow("Stabilize",outImg);

        if(waitKey(20) == 27)
        {
            cout<<"ESC key is pressed by user" <<endl;
            break;
        }







    }
}

}






