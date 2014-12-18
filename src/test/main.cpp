//#include "drone.hpp"

//using namespace drone;


//int main(int argc, char *argv[])
//{


//    Timer timerMain,timerFPS;
//   // timerFPS.startTimerFPS();
//   // timerMain.startTimer();
//    string outputPath="/home/sl001093/Documents/MAM5/PFE/videos/videoStabResult/morceau5stab.avi";
//    string path="/home/sl001093/Documents/MAM5/PFE/videos/Merio/merio9.avi";

//    /// STABILISATION
//    int method=IStabilization::ISTABILIZATION_KALMAN;
//    IStabilization Stabilisation;

//    Stabilisation.run(path,outputPath,method,false);

//}

///*
//    /// CAMSHIFT
//    CamshiftPerso cf;

//    bool skipFrame=false;


//    VideoCapture cap(path); // open the video file for reading


//    for(;;)
//    {



//        Mat frame;


//        cap>>frame;
//      //  cf.run(frame);






//     //   timerFPS.stopTimerFPS();
//       // timerFPS.getFPS();

//        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
//        {
//            cout << "esc key is pressed by user" << endl;
//            break;
//        }
//    }


//timerMain.stopTimer();
// timerMain.getTime();
//    return 0;


//}


///*
//    bool skipFrame=false;


//    string path="/home/sl001093/Documents/MAM5/PFE/videos/morceau3.avi";
//    VideoCapture cap(path); // open the video file for reading
//    Mat temp,temp1;
//    int stepFrame=10;

//    for(;;)
//    {

//        // fps counter begin
//        if (counter == 0){
//            time(&start);
//        }
//        Mat frame;
//        int nbreCurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);//Get the number of current frame
//        Debug::trace("Current frame: " + to_string(nbreCurrentFrame));


//        cap>>frame;
//        if (skipFrame){
//            if (nbreCurrentFrame%(stepFrame)==0){

//                temp = frame.clone();

//            }
//            if (nbreCurrentFrame%(stepFrame+3)==0){


//                temp1 = frame.clone();

//            }

//            namedWindow("frame");
//            imshow("frame",temp);
//        }
//        else {
//           //frame=VideoUtil::hidePartsVideo(frame);
//            frame=VideoUtil::geometricalCrop(frame,70,0);
//            namedWindow("frame");
//            imshow("frame",frame);
//        }

//        // fps counter begin
//        time(&end);
//        counter++;
//        sec = difftime(end, start);
//        fps = counter/sec;
//        if (counter > 30)
//            cout << "Fps: " << fps << endl;

//        // overflow protection
//        if (counter == (INT_MAX - 1000))
//            counter = 0;
//        // fps counter end

//        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
//        {
//            cout << "esc key is pressed by user" << endl;
//            break;
//        }
//    }

//    timerMain.stopTimer();
//    timerMain.getTime();
//    return 0;


//}
//*/
