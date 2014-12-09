#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "core/features/stabilizationOpenCv.hpp"
using namespace std;
using namespace cv;
using namespace cv::videostab;


namespace drone
{

StabilizationOpenCv::StabilizationOpenCv(string& path,string& outputPath,bool stdev, bool save,
                                         bool twoPass, bool mFilter, bool trime):stdev(stdev),save_motion(save),isTwoPass(twoPass),
    motionFilter(mFilter),est_trime(trime){

    this->path=path;
    this->outputPath=outputPath;

}



void StabilizationOpenCv::run()
{
    Timer fpsTime;
    fpsTime.startTimerFPS();

    Mat stabilizedFrame;

    VideoWriter writer(outputPath, CV_FOURCC('X','V','I','D'),11,stabilizedFrames->nextFrame().size());
    if ( !writer.isOpened() ) //if not initialize the VideoWriter successfully, exit the program
    {
        drone_error("ERROR: Failed to write the video");

    }
    while (!(stabilizedFrame = stabilizedFrames->nextFrame()).empty())
    {
        if (save_motion)

            saveMotionsIfNecessary();
        if (!outputPath.empty())
        {
            std::cerr << "Save motion path not empty" << std::endl;



            writer << stabilizedFrame;
        }

        fpsTime.stopTimerFPS();
        fpsTime.getFPS();
        imshow("stabilizedFrame", stabilizedFrame);
        char key = static_cast<char>(waitKey(3));

        if (key == 27){
            writer.release();
            break;}

    }
    writer.release();
    cout << "\nfinished\n";
}


void StabilizationOpenCv::saveMotionsIfNecessary()
{
    static bool areMotionsSaved = true;
    if (!areMotionsSaved)
    {
        IFrameSource *frameSource = static_cast<IFrameSource*>(stabilizedFrames);
        TwoPassStabilizer *twoPassStabilizer = dynamic_cast<TwoPassStabilizer*>(frameSource);
        if (twoPassStabilizer)
        {
            ofstream f(outputPath.c_str());
            const vector<Mat> &motions = twoPassStabilizer->motions();
            f << motions.size() << endl;
            for (size_t i = 0; i < motions.size(); ++i)
            {
                Mat_<float> M = motions[i];
                for (int l = 0, k = 0; l < 3; ++l)
                    for (int s = 0; s < 3; ++s, ++k)
                        f << M(l,s) << " ";
                f << endl;
            }
        }
        areMotionsSaved = true;
        cout << "motions are saved";
    }

}

void StabilizationOpenCv::init(){

    try
    {


        StabilizerBase *stabilizer;
        GaussianMotionFilter *motionFilter = 0;

        if (stdev)
        {
            motionFilter = new GaussianMotionFilter();
        }



        if (isTwoPass)
        {
            TwoPassStabilizer *twoPassStabilizer = new TwoPassStabilizer();
            if (est_trime)
                twoPassStabilizer->setEstimateTrimRatio(true);
            if (motionFilter)
                twoPassStabilizer->setMotionStabilizer(motionFilter);
            stabilizer = twoPassStabilizer;
        }
        else
        {
            OnePassStabilizer *onePassStabilizer= new OnePassStabilizer();
            if (motionFilter)
                onePassStabilizer->setMotionFilter(motionFilter);
            stabilizer = onePassStabilizer;
        }



        VideoFileSource *frameSource = new VideoFileSource(path);
        outputFps = frameSource->fps();
        stabilizer->setFrameSource(frameSource);
        cout << "frame count: " << frameSource->frameCount() << endl;

        PyrLkRobustMotionEstimator *motionEstimator = new PyrLkRobustMotionEstimator();
        //Type d'estimation
        motionEstimator->setMotionModel(TRANSLATION);

        //Outlier ratio
        RansacParams ransacParams = motionEstimator->ransacParams();
        motionEstimator->setRansacParams(ransacParams);



        stabilizer->setLog(new LogToStdout());


        stabilizedFrames = dynamic_cast<IFrameSource*>(stabilizer);

        run();
    }
    catch (const exception &e)
    {
        cout << "error: " << e.what() << endl;
        stabilizedFrames.release();

    }
    stabilizedFrames.release();


}



}



