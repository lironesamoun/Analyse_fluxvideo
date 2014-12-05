#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "core/features/stabilizationLive.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;

//Declare Kalman Filter
 KalmanFilter KF (4,2,0);
 Mat_<float> state (4,1);
 Mat_<float> measurement (2,1);
namespace drone
{

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }
    // "+"
    friend Trajectory operator+(const Trajectory &c1,const Trajectory  &c2){
        return Trajectory(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
    }
    //"-"
    friend Trajectory operator-(const Trajectory &c1,const Trajectory  &c2){
        return Trajectory(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
    }
    //"*"
    friend Trajectory operator*(const Trajectory &c1,const Trajectory  &c2){
        return Trajectory(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
    }
    //"/"
    friend Trajectory operator/(const Trajectory &c1,const Trajectory  &c2){
        return Trajectory(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
    }
    //"="
    Trajectory operator =(const Trajectory &rx){
        x = rx.x;
        y = rx.y;
        a = rx.a;
        return Trajectory(x,y,a);
    }

    double x;
    double y;
    double a; // angle
};
void StabilizationLive::init_kalman(double x, double y)
{

     KF.statePre.at<float>(0) = x;
     KF.statePre.at<float>(1) = y;
     KF.statePre.at<float>(2) = 0;
     KF.statePre.at<float>(3) = 0;

     KF.transitionMatrix = *(Mat_<float>(4,4) << 1,0,1,0,    0,1,0,1,     0,0,1,0,   0,0,0,1);
     KF.processNoiseCov = *(Mat_<float>(4,4) << 0.2,0,0.2,0,  0,0.2,0,0.2,  0,0,0.3,0,
                                                                                  0,0,0,0.3);
     setIdentity(KF.measurementMatrix);
     setIdentity(KF.processNoiseCov,Scalar::all(1e-6));
     setIdentity(KF.measurementNoiseCov,Scalar::all(1e-1));
     setIdentity(KF.errorCovPost, Scalar::all(.1));
}

Point2f StabilizationLive::kalman_predict_correct(double x, double y)
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



StabilizationLive::StabilizationLive(string &path){
    this->path=path;
    // For further analysis

}
void StabilizationLive::init(){

    // For further analysis
    ofstream out_transform("/home/user/Documents/MAM5/PFE/videos/prev_to_cur_transformation.txt");
    ofstream out_trajectory("/home/user/Documents/MAM5/PFE/videos/trajectory.txt");
    ofstream out_smoothed_trajectory("/home/user/Documents/MAM5/PFE/videos/smoothed_trajectory.txt");
    ofstream out_new_transform("/home/user/Documents/MAM5/PFE/videosnew_prev_to_cur_transformation.txt");

    VideoCapture cap(path);
    assert(cap.isOpened());

    Mat cur, cur_grey;
    Mat prev, prev_grey;

    cap >> prev;//get the first frame.ch
    cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    vector <TransformParam> prev_to_cur_transform; // previous to current
    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;
    // Step 2 - Accumulate the transformations to get the image trajectory
    vector <Trajectory> trajectory; // trajectory at all frames
    //
    // Step 3 - Smooth out the trajectory using an averaging window
    vector <Trajectory> smoothed_trajectory; // trajectory at all frames
    Trajectory X;//posteriori state estimate
    Trajectory	X_;//priori estimate
    Trajectory P;// posteriori estimate error covariance
    Trajectory P_;// priori estimate error covariance
    Trajectory K;//gain
    Trajectory	z;//actual measurement
    double pstd = 4e-3;//can be changed
    double cstd = 0.25;//can be changed
    Trajectory Q(pstd,pstd,pstd);// process noise covariance
    Trajectory R(cstd,cstd,cstd);// measurement noise covariance
    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector <TransformParam> new_prev_to_cur_transform;
    //
    // Step 5 - Apply the new transformation to the video
    //cap.set(CV_CAP_PROP_POS_FRAMES, 0);
    Mat T(2,3,CV_64F);

    int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
    VideoWriter outputVideo;
    outputVideo.open("compare.avi" , CV_FOURCC('X','V','I','D'), 24,cvSize(cur.rows, cur.cols*2+10), true);
    //
    int k=1;
    int max_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    Mat last_T;
    Mat prev_grey_,cur_grey_;

    while(true) {

        // fps counter end
        cap >> cur;
        if(cur.data == NULL) {
            break;
        }

        cvtColor(cur, cur_grey, COLOR_BGR2GRAY);

        // vector from prev to cur
        vector <Point2f> prev_corner, cur_corner;
        vector <Point2f> prev_corner2, cur_corner2;
        vector <uchar> status;
        vector <float> err;

        goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for(size_t i=0; i < status.size(); i++) {
            if(status[i]) {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        // translation + rotation only
        Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

        // in rare cases no transform is found. We'll just use the last known good transform.
        if(T.data == NULL) {
            last_T.copyTo(T);
        }

        T.copyTo(last_T);

        // decompose T
        double dx = T.at<double>(0,2);
        double dy = T.at<double>(1,2);
        double da = atan2(T.at<double>(1,0), T.at<double>(0,0));
        //
        //prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

        out_transform << k << " " << dx << " " << dy << " " << da << endl;
        //
        // Accumulated frame to frame transform
        x += dx;
        y += dy;
        a += da;
           std::cout << "accumulated x,y: (" << x << "," << y << ")" << endl;
        //trajectory.push_back(Trajectory(x,y,a));
        //
        out_trajectory << k << " " << x << " " << y << " " << a << endl;
        //
        z = Trajectory(x,y,a);
        //
        if(k==1){
            // intial guesses
            X = Trajectory(0,0,0); //Initial estimate,  set 0
            P =Trajectory(1,1,1); //set error variance,set 1

        }
        else
        {


            X_ = X; //X_(k) = X(k-1);
            P_ = P+Q; //P_(k) = P(k-1)+Q;
            // measurement update£¨correction£©
            K = P_/( P_+R ); //gain;K(k) = P_(k)/( P_(k)+R );
            X = X_+K*(z-X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k));
            P = (Trajectory(1,1,1)-K)*P_; //P(k) = (1-K(k))*P_(k);
        }
        //smoothed_trajectory.push_back(X);
        out_smoothed_trajectory << k << " " << X.x << " " << X.y << " " << X.a << endl;
        //-
        // target - current
        double diff_x = X.x - x;//
        double diff_y = X.y - y;
        double diff_a = X.a - a;

        dx = dx + diff_x;
        dy = dy + diff_y;
        da = da + diff_a;

        //new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
        //
        out_new_transform << k << " " << dx << " " << dy << " " << da << endl;
        //
        T.at<double>(0,0) = cos(da);
        T.at<double>(0,1) = -sin(da);
        T.at<double>(1,0) = sin(da);
        T.at<double>(1,1) = cos(da);

        T.at<double>(0,2) = dx;
        T.at<double>(1,2) = dy;

        Mat cur2;

        warpAffine(prev, cur2, T, cur.size());

        cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));



        //Remove dark part because of stabilization
       // cur2 = cur2(Range(50,cur2.rows-100),Range(50,cur2.cols-100));
        // Resize cur2 back to cur size, for better side by side comparison
        resize(cur2, cur2, cur.size());

        // Now draw the original and stablised side by side for coolness
        Mat canvas = Mat::zeros(cur.rows, cur.cols*2+10, cur.type());

        prev.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
        cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));

        // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
        if(canvas.cols > 1920) {
            resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
        }
        //outputVideo<<canvas;
        imshow("before and after", canvas);

        waitKey(10);
        //
        prev = cur.clone();//cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
        k++;




    }
}


}






