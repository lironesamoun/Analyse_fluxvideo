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

    //Crop video
    bool crop=true;

    //Number of interest points to detect
    int nb_max=100;

    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;

    // Different matrix
    Mat prevFrameColor,prevFrameGray, currentColorImg,currentGrayImg ,last_transformationmatrix;
     Mat transformMatrix(2,3,CV_64F);

    //Read the video
    VideoCapture cap(path);
    double fps_original = cap.get(CV_CAP_PROP_FPS);
    std::cout << "fps of video original: " << fps_original << std::endl;

    cap >> prevFrameColor;

    //Crop picture
    if (crop){
        VideoUtil::geometricalCrop(prevFrameColor,80,0);
    }
    cvtColor(prevFrameColor,currentGrayImg,CV_BGR2GRAY);
    prevFrameGray = currentGrayImg.clone();// previous gray frame

    // Get mask in order to compute features ( 4 little mask in the picture, see the function for more details)
    cv::Mat mask=computeMask(prevFrameGray,230,400);

   int border = HORIZONTAL_BORDER_CROP * prevFrameGray.rows / prevFrameGray.cols; // get the aspect ratio correct

    int compteur=0;

    // Init stockage
    vector<Point2f> cornersPrev;//Stock features of previous frame
    vector<Point2f> cornersPrev2;//Stock features of previous frame after remove bad points
    cornersPrev.reserve(nb_max);

    vector<Point2f> cornersCurr;//Stock features of current frame
    vector<Point2f> cornersCurr2;//Stock features of current frame after remove bad points
    cornersCurr.reserve(nb_max);

    // TermCriteria for optical Flow method
    TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);

    /// Compute good features to track
    cv::goodFeaturesToTrack(prevFrameGray,cornersPrev,nb_max,0.01,5.0,prevFrameGray);

   /// Improve the accuracy of features
    cornerSubPix(prevFrameGray,cornersPrev,Size(15,15),Size(-1,-1),opticalFlowTermCriteria);

    // For all the frame
    for (;;){

        //Get the current colour frame
        cap >> currentColorImg;

        if (crop){
            VideoUtil::geometricalCrop(currentColorImg,80,0);
        }

       cvtColor(currentColorImg,currentGrayImg,CV_BGR2GRAY);
       cv::Mat currFrameGray=  currentGrayImg.clone();//Get the current frame to compare with the previous frame

       // cv::Mat mask=computeMask(prevFrameGray,230,400);


        vector<uchar> status;//output status vector (of unsigned chars); each element of the vector is set to 1
                             //if the flow for the corresponding features has been found, otherwise, it is set to 0.
        vector<float> errors;//output vector of errors; each element of the vector is set to an error for the corresponding feature,
                             //type of the error measure can be set in flags parameter; if the flow wasn’t found then the error is not defined
                            //(use the status parameter to find such cases).

        /// Track feature by computing the optical flow
        calcOpticalFlowPyrLK(prevFrameGray,currFrameGray,cornersPrev,cornersCurr,status,errors,Size(20,20),3,
                             opticalFlowTermCriteria);


        // Remove bad matches
        for(size_t i=0; i < status.size(); i++) {
            double motion = sqrt(pow(cornersPrev.at(i).x-cornersCurr.at(i).x,2)+pow(cornersPrev.at(i).y-cornersCurr.at(i).y,2));
            if(status[i]) {
                cornersPrev2.push_back(cornersPrev[i]);
                cornersCurr2.push_back(cornersCurr[i]);
            }
        }

        /// If the number of features decrease, we re-compute features to track them (same step as above)
        if (cornersPrev2.size()<nb_max-nb_max/3){
            cornersCurr2.clear();
            cornersPrev2.clear();
            cv::goodFeaturesToTrack(prevFrameGray,cornersPrev,nb_max,0.01,5.0,prevFrameGray);

            TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);

            cornerSubPix(prevFrameGray,cornersPrev,Size(15,15),Size(-1,-1),opticalFlowTermCriteria);
            calcOpticalFlowPyrLK(prevFrameGray,currFrameGray,cornersPrev,cornersCurr,status,errors,Size(20,20),3,
                                 opticalFlowTermCriteria);
            for(size_t i=0; i < status.size(); i++) {
                double motion = sqrt(pow(cornersPrev.at(i).x-cornersCurr.at(i).x,2)+pow(cornersPrev.at(i).y-cornersCurr.at(i).y,2));
                if(status[i]) {
                    cornersPrev2.push_back(cornersPrev[i]);
                    cornersCurr2.push_back(cornersCurr[i]);
                }
            }

        }

        Debug::trace("Number of track feature cornerprev found: " + to_string(cornersPrev2.size()));


        /// Draw features
        cv::Mat drawingPoint;
        currentColorImg.copyTo(drawingPoint);
        //cvtColor(drawingPoint,drawingPoint,CV_GRAY2RGB);
        for( int i = 0; i < (int)cornersPrev2.size(); i++ )
        {

                line (drawingPoint, cornersPrev2.at(i),
                      cornersCurr2.at(i),cv::Scalar(0,0,255));
            cv::circle(drawingPoint,cornersCurr2.at(i), 3, cv::Scalar(0,0,255),-1);
        }

        std::cout << "Test " << std::endl;

        namedWindow("drawingPoint");
        imshow("drawingPoint",drawingPoint);

        /// Transformation matrix: estimate the rigid transform between the two frame
        transformMatrix = estimateRigidTransform(cornersPrev2,cornersCurr2 ,false);

        // If the transformation matrix is not found
        if(transformMatrix.data == NULL) {
            last_transformationmatrix.copyTo(transformMatrix);
        }

        transformMatrix.copyTo(last_transformationmatrix);

        /// Smoothing part

        // decompose T
        double dx = transformMatrix.at<double>(0,2);
        double dy = transformMatrix.at<double>(1,2);
        double da = atan2(transformMatrix.at<double>(1,0), transformMatrix.at<double>(0,0));

        // Accumulated frame to frame transform
        x += dx;
        y += dy;
        a += da;
        Debug::info("accumulated x,y: ("+to_string(x) + "," + to_string(y) + ")");

        //If first i on the loop, we initialize kalman filter
        if (compteur==0){
            init_kalman(x,y);
        }
        // Prediction step
        else {

              vector<Point2f> smooth_feature_point;
              Point2f smooth_feature=kalman_predict_correct(x,y);
              smooth_feature_point.push_back(smooth_feature);
              Debug::trace("smooth x,y: ("+to_string(smooth_feature.x) + "," + to_string(smooth_feature.y) + ")");


              // target - current
              double diff_x = smooth_feature.x - x;//
              double diff_y = smooth_feature.y - y;

              dx = dx + diff_x;
              dy = dy + diff_y;

              //Reconstruction of the matrix
              transformMatrix.at<double>(0,0) = cos(da);
              transformMatrix.at<double>(0,1) = -sin(da);
              transformMatrix.at<double>(1,0) = sin(da);
              transformMatrix.at<double>(1,1) = cos(da);
              transformMatrix.at<double>(0,2) = dx;
              transformMatrix.at<double>(1,2) = dy;

              cv::Mat outImg;
              /// Application of the transformation with the new matrix
              warpAffine(prevFrameColor,outImg,transformMatrix,currentColorImg.size());
              //Crop the dark border du to the stabilization
              outImg = outImg(Range(border, outImg.rows-border), Range(HORIZONTAL_BORDER_CROP, outImg.cols-HORIZONTAL_BORDER_CROP));


              //Display
              namedWindow("stabilized");
              imshow("stabilized",outImg);

              /// change the frame
              prevFrameColor= currentColorImg.clone();
              currentGrayImg.copyTo(prevFrameGray);


        }

        // Delete the features, in order to re compute nb_max again
        cornersPrev2.clear();

        cornersCurr2.clear();


        compteur++;

        // get the fps
        timerFPS.stopTimerFPS();
        timerFPS.getFPS();


        if(waitKey(27) >= 0) break;

    }
}

}




