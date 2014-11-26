#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include "fstream"



using namespace cv;



int main(int argc, char *argv[])
{

    std::cout << "Hello coucou " << std::endl;
    cv::Mat imgTest= cv::imread("/home/sl001093/Bureau/arc.png");
    cv::imshow("test",imgTest);
    cv::waitKey(0);

}

