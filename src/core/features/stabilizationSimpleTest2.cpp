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
//Declare Kalman Filter
 KalmanFilter KF (4,2,0);
 Mat_<float> state (4,1);
 Mat_<float> measurement (2,1);

StabilizationTestSimple2::StabilizationTestSimple2(string &path){
    this->path=path;

    // For further analysis


}

cv::Mat StabilizationTestSimple2::computeMask(Mat& frame,int lh,int lw){
    int height=frame.rows;
    int width=frame.cols;

    /// Creation du masque pour le calcul de point d'interet
    int limitH=lh;
    int limitW=lw;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);  //NOTE: using the type explicitly    //Bas en haut puis gaudre à droite
    cv::Mat roi1=frame(Range(50,limitH),Range(50,limitW));//Coin haut à gauche
    cv::Mat roi2=frame(Range(height-limitH,height-50),Range(50,limitW));//Coinbas à gauche
    cv::Mat roi3=frame(Range(50,limitH),Range(width-limitW,width-50));//Coin haut à droite
    cv::Mat roi4=frame(Range(height-limitH,height-50),Range(width-limitW,width-50));//Coin bas à droite

    roi1.copyTo(mask(Range(50,limitH),Range(50,limitW)));
    roi2.copyTo(mask(Range(height-limitH,height-50),Range(50,limitW)));
    roi3.copyTo(mask(Range(50,limitH),Range(width-limitW,width-50)));
    roi4.copyTo(mask(Range(height-limitH,height-50),Range(width-limitW,width-50)));



    return mask;

}

void StabilizationTestSimple2::init_kalman(double x, double y)
{

     KF.statePre.at<float>(0) = x;
     KF.statePre.at<float>(1) = y;
     KF.statePre.at<float>(2) = 0;
     KF.statePre.at<float>(3) = 0;

     KF.transitionMatrix = *(Mat_<float>(4,4) << 1,0,1,0,    0,1,0,1,     0,0,1,0,   0,0,0,1);
     KF.processNoiseCov = *(Mat_<float>(4,4) << 0.2,0,0.2,0,  0,0.2,0,0.2,  0,0,0.3,0,
                                                                                  0,0,0,0.3);
     setIdentity(KF.measurementMatrix);
     setIdentity(KF.processNoiseCov,Scalar::all(1e-4));
     setIdentity(KF.measurementNoiseCov,Scalar::all(1e-1));
     setIdentity(KF.errorCovPost, Scalar::all(.1));
      measurement.setTo(Scalar(0));
}

Point2f StabilizationTestSimple2::kalman_predict_correct(double x, double y)
{
     Mat prediction = KF.predict();
     Point2f predictPt (prediction.at<float>(0), prediction.at<float>(1));
     std::cout << "predicted x,y: (" << predictPt.x << "," << predictPt.y << ")" << endl;
     measurement(0) = x;
     measurement(1) = y;
     Mat estimated = KF.correct(measurement);
     Point2f statePt (estimated.at<float>(0), estimated.at<float>(1));
     return statePt;
}

void StabilizationTestSimple2::init(){

    Timer timerFPS;
    timerFPS.startTimerFPS();

    //Number of interest points to detect
    int nb_max=50;

    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;

    // Different matrix
    Mat prevFrame, colorImg, outImg, grayImg,last_transformationmatrix;

    //Read the video
    VideoCapture cap(path);
    double fps_original = cap.get(CV_CAP_PROP_FPS);
    std::cout << "fps of video original: " << fps_original << std::endl;


    cap.read(colorImg);
    VideoUtil::geometricalCrop(colorImg,80,0);//Crop the picture
    cvtColor(colorImg,grayImg,CV_BGR2GRAY);
    prevFrame = grayImg.clone();// Current picture

    // Get mask in order to compute features
    cv::Mat mask=computeMask(prevFrame,230,400);

    Mat currFrame=prevFrame;


    int compteur=0;



    for (;;){

        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame

        currFrame.copyTo(prevFrame);//Get the reference (previous) frame
        cap.read(colorImg);
        cv::Mat originalFrame=colorImg.clone();
        VideoUtil::geometricalCrop(colorImg,80,0);
        cvtColor(colorImg,grayImg,CV_BGR2GRAY);
        currFrame=  grayImg.clone();//Get the current frame to compare with the previous frame


        cv::Mat mask=computeMask(prevFrame,230,400);
        namedWindow("Mask");
        imshow("Mask",mask);

        /// Calcul de points d'interet
        vector<Point2f> cornersPrev,cornersPrev2;//Stock features of reference Frame
        cornersPrev.reserve(nb_max);

        vector<Point2f> cornersCurr,cornersCurr2;
        cornersCurr.reserve(nb_max);
        cv::goodFeaturesToTrack(prevFrame,cornersPrev,nb_max,0.01,5.0,prevFrame);

        TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);


        cornerSubPix(prevFrame,cornersPrev,Size(15,15),Size(-1,-1),opticalFlowTermCriteria);


        vector<uchar> features_found;
        vector<float> features_error;
        // calcOpticalFlowPyrLK(prevFrame,currFrame,cornersPrev,cornersCurr,features_found,features_error,Size(10∗4+1,10∗4+1),5,opticalFlowTermCriteria);
        calcOpticalFlowPyrLK(prevFrame,currFrame,cornersPrev,cornersCurr,features_found,features_error,Size(20,20),3,
                             opticalFlowTermCriteria);


        // weed out bad matches
        for(size_t i=0; i < features_found.size(); i++) {
            if(features_found[i]) {
                cornersPrev2.push_back(cornersPrev[i]);
                cornersCurr2.push_back(cornersCurr[i]);
            }
        }
        /*
        // keep the good points
        std::vector<cv::Point2f> initialPoints;
        std::vector<cv::Point2f> trackedPoints;
        for (int i=0;i<cornersCurr.size();i++){
            double motion = sqrt(pow(cornersPrev.at(i).x-cornersCurr.at(i).x,2)+pow(cornersPrev.at(i).y-cornersCurr.at(i).y,2));
            // std::cout << "Motion: " << motion << std::endl;
            if (features_found[i] && motion < 20 ){
                //Keep this point in vector
                initialPoints.push_back(cornersPrev.at(i));
                trackedPoints.push_back(cornersCurr.at(i));
            }
        }
*/
        /// Draw features
        Debug::info("Size features found: " + to_string(features_found.size()));
        cv::Mat drawingPoint;
        currFrame.copyTo(drawingPoint);
        cvtColor(drawingPoint,drawingPoint,CV_GRAY2RGB);
        for( int i = 0; i < (int)cornersPrev2.size(); i++ )
        {

                line (drawingPoint, cornersPrev2.at(i),
                      cornersCurr2.at(i),cv::Scalar(0,0,255));
            cv::circle(drawingPoint,cornersCurr2.at(i), 3, cv::Scalar(0,0,255),-1);
        }


        namedWindow("drawingPoint");
        imshow("drawingPoint",drawingPoint);
        /// Transformation
        Mat transformMatrix = estimateRigidTransform(cornersPrev2,cornersCurr2 ,false);

        // in rare cases no transform is found. We'll just use the last known good transform.
        if(transformMatrix.data == NULL) {
            last_transformationmatrix.copyTo(transformMatrix);
        }

        transformMatrix.copyTo(last_transformationmatrix);

        // decompose T
        double dx = transformMatrix.at<double>(0,2);
        double dy = transformMatrix.at<double>(1,2);
        double da = atan2(transformMatrix.at<double>(1,0), transformMatrix.at<double>(0,0));

        // Accumulated frame to frame transform
        x += dx;
        y += dy;
        a += da;
        std::cout << "accumulated x,y: (" << x << "," << y << ")" << endl;

        if (compteur==0){
            init_kalman(x,y);
        }
        else {


              vector<Point2f> smooth_feature_point;
              Point2f smooth_feature=kalman_predict_correct(x,y);
              smooth_feature_point.push_back(smooth_feature);
              std::cout << "smooth x,y: (" << smooth_feature.x << "," << smooth_feature.y << ")" << endl;

              // target - current
              double diff_x = smooth_feature.x - x;//
              double diff_y = smooth_feature.y - y;

              dx = dx + diff_x;
              dy = dy + diff_y;

              transformMatrix.at<double>(0,0) = cos(da);
              transformMatrix.at<double>(0,1) = -sin(da);
              transformMatrix.at<double>(1,0) = sin(da);
              transformMatrix.at<double>(1,1) = cos(da);
              transformMatrix.at<double>(0,2) = dx;
              transformMatrix.at<double>(1,2) = dy;

              warpAffine(prevFrame,outImg,transformMatrix,prevFrame.size());

              namedWindow("stabilized");
              imshow("stabilized",outImg);
              namedWindow("Original");
              imshow("Original",mask);
              //waitKey(0);


        }
         compteur++;






        //std::cout << "type: " << VideoUtil::type2str(transformMatrix.type()) << std::endl;
        //warpAffine(currFrame,outImg,transformMatrix,currFrame.size(), INTER_NEAREST|WARP_INVERSE_MAP, BORDER_CONSTANT);
      //  Mat transformMatrix = estimateGlobalMotionRobust(cornersPrev,cornersCurr,3,RansacParams::affine2dMotionStd(),0,0);


       // warpPerspective(currFrame,outImg,transformMatrix,currFrame.size(), INTER_LINEAR |WARP_INVERSE_MAP,BORDER_CONSTANT ,0);

       // namedWindow("Features before");
        //imshow("Features before",drawingPoint);


        /// Smoothing motion
        // First predict, to update the internal statePre variable

        //For each frame, we have to predict all the keypoints
        //Initialization
       /* for (int i=0;i<cornersCurr.size();i++){
               init_kalman(cornersCurr.at(i).x,cornersCurr.at(i).y);
        }
        for (int i=0;i<cornersCurr.size();i++){
            Point2f smooth_feature=kalman_predict_correct(cornersCurr.at(i).x,cornersCurr.at(i).y);
            smooth_feature.x=(int)smooth_feature.x;
            smooth_feature.y=(int)smooth_feature.y;
            smooth_feature_point.push_back(smooth_feature);
            std::cout << "smooth x,y: (" << smooth_feature.x << "," << smooth_feature.y << ")" << endl;
        }
*/
         //Mat transformMatrix = estimateRigidTransform(cornersPrev,smooth_feature_point ,false);

       //   Mat transformMatrix = estimateGlobalMotionRobust(cornersPrev,smooth_feature_point,3,RansacParams::affine2dMotionStd(),0,0);

       //    std::cout << "type: " << VideoUtil::type2str(transformMatrix.type()) << std::endl;

       // warpAffine(currFrame,outImg,transformMatrix,currFrame.size(), INTER_NEAREST|WARP_INVERSE_MAP, BORDER_CONSTANT);

         //  warpPerspective(currFrame,outImg,transformMatrix,currFrame.size(), INTER_LINEAR |WARP_INVERSE_MAP,BORDER_CONSTANT ,0);




        /*// Now draw the original and stablised side by side for coolness
        Mat canvas = Mat::zeros(currFrame.rows, currFrame.cols*2+10, currFrame.type());

        originalFrame.copyTo(canvas(Range::all(), Range(0, outImg.cols)));
        outImg.copyTo(canvas(Range::all(), Range(outImg.cols+10, outImg.cols*2+10)));

        // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
        if(canvas.cols > 1920) {
            resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
        }
        //outputVideo<<canvas;
        imshow("before and after", canvas);*/


        //  cap.read(colorImg);






        namedWindow("Original");
        imshow("Original",originalFrame);

        timerFPS.stopTimerFPS();
        timerFPS.getFPS();


        if(waitKey(27) >= 0) break;

    }
}

}




