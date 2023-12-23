#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <ros/ros.h>

using namespace cv;
using namespace std;

void salt(cv::Mat &image, int n){

    int i, j;
    for(int k = 0; k<n; k++){

        i = rand() % image.cols;
        j = rand() % image.rows;

        if (image.channels()== 1){

            image.at<uchar>(j, i) =255;
        }
        else if (image.channels() == 3){

            image.at<cv::Vec3b>(j, i)[0] = 255;
            image.at<cv::Vec3b>(j, i)[1] = 255;
            image.at<cv::Vec3b>(j, i)[2] = 255;
        }
    }
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "point");


    srand(cv::getTickCount());

    cv::Mat image = cv::imread("image.png", 1);
    
    Mat dst, cdst;
    Canny(image, dst, 50, 150, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    
    // cv::imshow("canny", cdst);

    salt(cdst, 3000);

    cv::namedWindow("Image",1);
    cv::imshow("Image", cdst);
    cv::imwrite("salted.bmp", cdst);

    cv::waitKey(5000);

    ros::spin();
  
    return 0;
}
