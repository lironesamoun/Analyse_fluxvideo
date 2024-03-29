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
#include "core/features/stabilizationKalman.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;


namespace drone
{

//Declare Kalman Filter
// >>>> Kalman Filter
int stateSize = 3;
int measSize = 3;
int contrSize = 0;

unsigned int type = CV_32F;
cv::KalmanFilter KF(stateSize, measSize, contrSize, type);
cv::Mat state(stateSize, 1, type);  // [x,y,teta]
cv::Mat measurement(measSize, 1, type);    // [z_x,z_y,z_teta]
//  KalmanFilter KF (4,2,0);
//Mat_<float> state (4,1);
//Mat_<float> measurement (2,1);

StabilizationKalman::StabilizationKalman(string &path,string& outputP,bool save):path(path),outputPath(outputP),save(save)
{

}


/*
void StabilizationKalman::init_kalman(double x, double y)
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
*/
void StabilizationKalman::init_kalman1(double x, double y,double teta)
{

    cv::setIdentity(KF.transitionMatrix);
    cv::setIdentity(KF.measurementMatrix);
    Debug::trace("Kalman: transition and measurement matrix okey");
    KF.statePre.at<float>(0) = x;
    KF.statePre.at<float>(1) = y;
    KF.statePre.at<float>(2) = teta;

    Debug::trace("Kalman:state okey");
    setIdentity(KF.processNoiseCov,Scalar::all(1e-2));
    // Measures Noise Covariance Matrix R
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
    measurement.setTo(Scalar(0));
    Debug::trace("Kalman: measurement 0 okey");
}

Point3f StabilizationKalman::kalman_predict_correct1(double x, double y,double teta)
{
    Mat prediction = KF.predict();
    Point3f predictPt (prediction.at<float>(0), prediction.at<float>(1),prediction.at<float>(2));
    std::cout << "predicted x,y,teta: (" << predictPt.x << "," << predictPt.y << ","  << predictPt.z << ")" << endl;
    measurement.at<float>(0) = x;
    measurement.at<float>(1) = y;
    measurement.at<float>(2) = teta;
    Mat estimated = KF.correct(measurement);
    Point3f statePt (estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));
    return statePt;
}

/*
Point2f StabilizationKalman::kalman_predict_correct(double x, double y)
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
*/
void StabilizationKalman::init(){

    Timer timerFPS;
    timerFPS.startTimerFPS();

    //Crop video
    bool crop=true;

    //If kalman filter is init
    bool KalmanHasInit=false;

    //Number of interest points to detect
    int nb_max=100;

    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;

    // Different matrix
    Mat prevFrameColor,prevFrameGray, currentColorImg,currentGrayImg ,last_transformationmatrix;
    Mat transformMatrix= Mat::zeros(2,3,CV_64F);


    //Read the video
    VideoCapture cap(path);
    int nbre_frame=cap.get(CV_CAP_PROP_FRAME_COUNT);

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video


    double fps_original = cap.get(CV_CAP_PROP_FPS);
    std::cout << "fps of video original: " << fps_original << std::endl;

    cap >> prevFrameColor;

    //Crop picture
    if (crop){
        VideoUtil::geometricalCrop(prevFrameColor,80,0);
    }
    cvtColor(prevFrameColor,currentGrayImg,CV_BGR2GRAY);
    prevFrameGray = currentGrayImg.clone();// previous gray frame


    // Size frameSize(static_cast<int>(prevFrameColor.cols*2+10), static_cast<int>(prevFrameColor.rows));
    VideoWriter video;
    if (save){
        Size frameSize(static_cast<int>(prevFrameColor.cols*2+10), static_cast<int>(prevFrameColor.rows));
        video =VideoWriter (outputPath,CV_FOURCC('D', 'I', 'V', 'X'),20, frameSize,true);

        if ( !video.isOpened() || outputPath.empty()) //if not initialize the VideoWriter successfully, exit the program
        {
            imageretrieval_error("ERROR: Failed to write the video");

        }

    }


    // Get mask in order to compute features ( 4 little mask in the picture, see the function for more details)
    cv::Mat mask=VideoUtil::computeMask(prevFrameGray,230,400);

  //  int border = HORIZONTAL_BORDER_CROP * prevFrameGray.rows / prevFrameGray.cols; // get the aspect ratio correct
    int border=1;
    int compteur=0;

    // Init stockage
    vector<Point2f> featuresPrevFrame;//Stock features of previous frame
    vector<Point2f> featuresPrevFrame2;//Stock features of previous frame after remove bad points
    featuresPrevFrame.reserve(nb_max);

    vector<Point2f> featuresCurrentFrame;//Stock features of current frame
    vector<Point2f> featuresCurrentFrame2;//Stock features of current frame after remove bad points
    featuresCurrentFrame.reserve(nb_max);

    // TermCriteria for optical Flow method
    TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);

    /// Compute good features to track
    cv::goodFeaturesToTrack(prevFrameGray,featuresPrevFrame,nb_max,0.01,5.0,prevFrameGray);

    /// Improve the accuracy of features
    cornerSubPix(prevFrameGray,featuresPrevFrame,Size(15,15),Size(-1,-1),opticalFlowTermCriteria);


    // For all the frame
    while(compteur<nbre_frame){
        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame


        //Get the current colour frame
        cap >> currentColorImg;
        if(currentColorImg.data == NULL) {
            break;
        }


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
        calcOpticalFlowPyrLK(prevFrameGray,currFrameGray,featuresPrevFrame,featuresCurrentFrame,status,errors,Size(20,20),3,
                             opticalFlowTermCriteria);



        // Remove bad matches
        for(size_t i=0; i < status.size(); i++) {
            double motion = sqrt(pow(featuresPrevFrame.at(i).x-featuresCurrentFrame.at(i).x,2)+pow(featuresPrevFrame.at(i).y-featuresCurrentFrame.at(i).y,2));
            if(status[i]) {
                featuresPrevFrame2.push_back(featuresPrevFrame[i]);
                featuresCurrentFrame2.push_back(featuresCurrentFrame[i]);
            }
        }

        /// If the number of features decrease, we re-compute features to track them (same step as above)
        if (compteur%50==0){
            std::cout << "okey " << std::endl;
            featuresCurrentFrame2.clear();
            featuresPrevFrame2.clear();
            cv::goodFeaturesToTrack(prevFrameGray,featuresPrevFrame,nb_max,0.01,5.0,prevFrameGray);

            TermCriteria opticalFlowTermCriteria = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,20,0.3);

            cornerSubPix(prevFrameGray,featuresPrevFrame,Size(15,15),Size(-1,-1),opticalFlowTermCriteria);
            calcOpticalFlowPyrLK(prevFrameGray,currFrameGray,featuresPrevFrame,featuresCurrentFrame,status,errors,Size(20,20),3,
                                 opticalFlowTermCriteria);

            for(size_t i=0; i < status.size(); i++) {
                double motion = sqrt(pow(featuresPrevFrame.at(i).x-featuresCurrentFrame.at(i).x,2)+pow(featuresPrevFrame.at(i).y-featuresCurrentFrame.at(i).y,2));
                if(status[i]) {
                    featuresPrevFrame2.push_back(featuresPrevFrame[i]);
                    featuresCurrentFrame2.push_back(featuresCurrentFrame[i]);
                }
            }

        }

        Debug::trace("Number of track feature cornerprev found: " + to_string(featuresPrevFrame2.size()));
        /* namedWindow("Frame");
        imshow("Frame",currFrameGray);
        waitKey(0);*/

        if (!featuresPrevFrame2.empty()){

            /// Draw features
            cv::Mat drawingPoint;
            currentColorImg.copyTo(drawingPoint);
            //cvtColor(drawingPoint,drawingPoint,CV_GRAY2RGB);
            for( int i = 0; i < (int)featuresPrevFrame2.size(); i++ )
            {

                line (drawingPoint, featuresPrevFrame2.at(i),
                      featuresCurrentFrame2.at(i),cv::Scalar(0,0,255));
                cv::circle(drawingPoint,featuresCurrentFrame2.at(i), 3, cv::Scalar(0,0,255),-1);
            }



            namedWindow("drawingPoint");
            imshow("drawingPoint",drawingPoint);

            /// Transformation matrix: estimate the rigid transform between the two frame
            transformMatrix = estimateRigidTransform(featuresPrevFrame2,featuresCurrentFrame2 ,false);

            // If the transformation matrix is not found
            if(transformMatrix.data == NULL) {
                if (last_transformationmatrix.data==NULL){
                    Mat transformMatrix= Mat::zeros(2,3,CV_64F);
                    Debug::trace("Dans IF: " + to_string(transformMatrix.size()));
                }
                else {
                    Debug::trace("Transformation matrix null avant");
                    last_transformationmatrix.copyTo(transformMatrix);
                    Debug::trace("Transformation matrix null après");
                }
            }

            transformMatrix.copyTo(last_transformationmatrix);


            /// Smoothing part

            // decompose T
            if (!transformMatrix.empty()){



                double dx = transformMatrix.at<double>(0,2);
                double dy = transformMatrix.at<double>(1,2);
                double da = atan2(transformMatrix.at<double>(1,0), transformMatrix.at<double>(0,0));
                Debug::trace("Donnés recupéres: ");

                // Accumulated frame to frame transform
                x += dx;
                y += dy;
                a += da;
                Debug::info("accumulated x,y,teta: ("+to_string(x) + "," + to_string(y) + "," + to_string(a) + ")");

                //If first i on the loop, we initialize kalman filter
                if (!KalmanHasInit){
                    // init_kalman(x,y);
                    Debug::trace("Init Kalman");
                    init_kalman1(x,y,a);
                    KalmanHasInit=true;
                }
                // Prediction step
                else {

                    // vector<Point2f> smooth_feature_point;
                    //Point2f smooth_feature=kalman_predict_correct(x,y);
                    vector<Point3f> smooth_feature_point;
                    Point3f smooth_feature=kalman_predict_correct1(x,y,a);
                    smooth_feature_point.push_back(smooth_feature);
                    Debug::trace("smooth x,y,teta: ("+to_string(smooth_feature.x) + "," + to_string(smooth_feature.y) +  "," + to_string(smooth_feature.z) +")");


                    // target - current
                    double diff_x = smooth_feature.x - x;//
                    double diff_y = smooth_feature.y - y;
                    double diff_a = smooth_feature.z - a;

                    dx = dx + diff_x;
                    dy = dy + diff_y;
                    da = da + diff_a;

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
                    // namedWindow("stabilized");
                    //imshow("stabilized",outImg);

                    /// change the frame
                    prevFrameColor= currentColorImg.clone();
                    currentGrayImg.copyTo(prevFrameGray);

                    resize(outImg, outImg, currentColorImg.size());

                    // Now draw the original and stablised side by side for coolness
                    Mat canvas = Mat::zeros(currentColorImg.rows, currentColorImg.cols*2+10, currentColorImg.type());

                    prevFrameColor.copyTo(canvas(Range::all(), Range(0, outImg.cols)));
                    outImg.copyTo(canvas(Range::all(), Range(outImg.cols+10, outImg.cols*2+10)));

                    // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
                    /*     if(canvas.cols > 1920) {
                resize(canvas, canvas, Size(canvas.cols/1.5, canvas.rows));
            }
*/


                    namedWindow("Stabilization");
                    imshow("Stabilization",canvas);


                    //  video.write(canvas);
                    if (save){

                        //video << outImg;
                        //video.write(canvas);
                       video << canvas;
                        Debug::trace("Save");
                    }


                }
            }
        }
        // Delete the features, in order to re compute nb_max again
        featuresPrevFrame2.clear();

        featuresCurrentFrame2.clear();


        compteur++;


        // get the fps
        timerFPS.stopTimerFPS();
        timerFPS.getFPS();

        std::cout << "frame: " << nbreCurrentFrame << "/" << cap.get(CV_CAP_PROP_FRAME_COUNT) << std::endl;
        /*if (nbreCurrentFrame>cap.get(CV_CAP_PROP_FRAME_COUNT) -5)
        {
            break;
        }*/


        waitKey(1000);
        if(waitKey(27) >= 0) break;


    }
    video.release();
}

}




