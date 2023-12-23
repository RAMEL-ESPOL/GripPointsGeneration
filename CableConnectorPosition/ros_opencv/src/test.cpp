#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

#define ABS(a)      (((a)<(0))?-(a):(a))
//#define MAX(a,b)    (((a)>(b))?(a):(b)) // already defined by openCV
#define Deg2Rad 0.01745329251 // pi/180
#define Rad2Deg 57.295779546 // 180/pi
#define MAX_RPM 300

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    String filename("/home/francisco/catkin_ws/src/cable-connector-recognition/ros_opencv/src/sek.jpg");

    Mat src = imread(filename, 0);

 if(src.empty())
 {

     cout << "can not open " << filename << endl;
     return -1;
 }


cout << "Width : " << src.size().width << endl;
cout << "Height: " << src.size().height << endl;
//Size size(700,813);//the dst image size,e.g.100x100

//Mat src;//src image
//resize(src,dist,size);//resize image

//Mat image;
/*Rect myROI = Rect(245, 5, 175, 765);
Mat cropedImage = src(myROI);
//Mat cropedImage = dist(Rect(245,5,175,765));
imshow("source", cropedImage);*/



 Mat dst, result;
 Canny(src, dst, 50, 150, 3);
 cvtColor(dst, result, CV_GRAY2BGR);


 //Ransac

   vector<Vec4i> lines;

        //////////////////////////////////
        float inlierScore;
        float A,  B,  C;
        float A_, B_, C_;
        float theta;
        float temp;
        int poiSize;
        int lineSize;
        float degree;
        vector<Point> poi;
        Point pS, pE; // start, end
        pS.x = 0; pS.y = 0;
        pE.x = 0; pE.y = 0;

        int P = 3;
        int N = 7; // N = log(1-p)/log(1-u^m)

// RANSAC
        // Points of interest
        for (int y=0; y<result.rows; y++)
        {
            Vec3b* res = result.ptr<Vec3b>(y);
            for (int x=0; x<result.cols; x++)// x=320
            {
                if (((int)res[x][0])!=0)
                    poi.push_back(Point(x,y));
            }
        }
        poiSize = poi.size();

        cout << poiSize << endl;
        if (poiSize > 100)
///////////////////////////////////
{
            // iteration
            for (int i=0; i<P; i++)
            {
                inlierScore = 0;
                for (int j=0; j<N; j++)
                {
                    // select 2 random points
                    int a = rand()%poiSize;
                    int b;
                    do {b = rand()%poiSize;} while(a==b);
                    Point p1 = poi[a];
                    Point p2 = poi[b];

                    int dx = p2.x-p1.x;
                    int dy = p2.y-p1.y;
                    float length = sqrt(dx*dx+dy*dy);

                    if (length > 50)
                        ////////////////////////////////////
                    {
                        // (y-y1)/(y2-y1) = (x-x1)/(x2-x1)*
                        // -(y2-y1)*x + (x2-x1)*y  + (y2-y1)*x1-(x2-x1)*y1 = 0
                        // Ax + By + C = 0
                        // A = -(y2-y1)
                        // B = (x2-x1)
                        // C = -A*x1 - B*y1
                        // distance = ABS(A*x + B*y + C)/SQRT(A^2+B^2)
                        A = (float)(p1.y - p2.y);
                        B = (float)(p2.x - p1.x);
                        C = (-A)*((float)p1.x) - (B)*((float)p1.y);

                        // check inlier
                        float score = 0;
                        for(int k=0; k<poiSize; k++)
                        {
                            Point p = poi[k];
                            float distance = abs(A*p.x + B*p.y + C)/sqrt(A*A+B*B);
                            if (distance < 10) // 10 pixels
                            {
                                if (distance < 2)    /////////////////////////////////////////
                                    score += 10;
                                else
                                    score += 0.1;
                            }
                        }
                        if (score > inlierScore)
                        {/*
                            A_ = A;
                            B_ = B;
                            C_ = C;*/
                            pS.x = p1.x; pS.y = p1.y;
                            pE.x = p2.x; pE.y = p2.y;
                            inlierScore = score;
                        }
                    }
                }
//                line(result, pS, pE, Scalar(0, 255, 0), 2, CV_AA);
//                line(topview, pS, pE, Scalar(0, 255, 0), 2, CV_AA);
                lines.push_back(Vec4i(pS.x, pS.y, pE.x, pE.y));
            }
        }
        lineSize = lines.size();

        // Estimation
        inlierScore = 0;
        for (int i=0; i<lineSize; i++)
        {
            // find line parameters
            Vec4i Lp = lines[i];
            A = (float)(Lp[1] - Lp[3]);
            B = (float)(Lp[2] - Lp[0]);
            C = (-A)*((float)Lp[0]) - (B)*((float)Lp[1]);

            // check inlier
            float score = 0;
            for (int k=0; k<poiSize; k++)
            {
                Point p = poi[k];
                float distance = abs(A*p.x + B*p.y + C)/sqrt(A*A+B*B);
                if (distance < 10) // 10 pixels
                {
                    if (distance < 1.3)    /////////////////////////////////////////
                        score += 10;
                    else
                        score += 0.1;
                }
            }

            if (score > inlierScore)
            {
                A_ = A;
                B_ = B;
                C_ = C;
                pS.x = Lp[0]; pS.y = Lp[1];
                pE.x = Lp[2]; pE.y = Lp[3];
                inlierScore = score;
//                line(result, pS, pE, Scalar(((j*200)/N)+55, 0, 0), 2, CV_AA);
//                line(topview, pS, pE, Scalar(((j*200)/N)+55, 0, 0), 2, CV_AA);
            }
        }
        theta = atan2f(A_, B_); // atan2f(dy, dx)
        temp = theta * Rad2Deg;
        if (temp > 90)
            degree = temp - 180;
        else
            degree = temp;
        if (degree > 0)
            degree = (-degree)+90;
        else
            degree = (-degree)-90;
       line(result,  pS, pE, Scalar(0, 0, 255), 2, CV_AA);
       // line(topview, pS, pE, Scalar(0, 0, 255), 2, CV_AA);

        ////////////////////////////////////////////////////////////////


        std::ostringstream ss;
        ss << "Heading=" << degree;
        std::string headingText(ss.str());
        cv::putText(result, headingText, cv::Point(290,460), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 0, 255));
        //cv::putText(topview, headingText, cv::Point(290,460), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 0, 255));

     //   cv::putText(result,  fnumber, cv::Point(290,445), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 255, 0));
        //cv::putText(topview, fnumber, cv::Point(290,445), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 255, 0));

        cv::putText(result,  "Result", cv::Point(290,430), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 0, 255));
       // cv::putText(topview, "Result", cv::Point(200,430), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 0, 255));
       // cv::putText(result,  "Candiate", cv::Point(200,445), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(255, 0, 0));
       // cv::putText(topview, "Candiate", cv::Point(200,445), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(255, 0, 0));
      //  cv::putText(result,  "RANSAC", cv::Point(200,460), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 255, 0));
       // cv::putText(topview, "RANSAC", cv::Point(200,460), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 255, 0));

        /////////////////////////////////////////////////////////////////

       // imshow("source", topview);
        imshow("detected lines", result);
        imshow("source", src);

      /*  char c = (char)cv::waitKey(1);
        if (c == 'd')
            c = (char)cv::waitKey(1);
        if (c == 's')
            c = (char)cv::waitKey(9999999);
        if (c == 'q')
            break;*/

     waitKey();
	return 0;
}
/*
void readData_P(const FileStorage& node, Mat& cameraMat, Mat& distCoeffs)
{
    node["image_width"] >> imageWidth;
    node["image_height"] >> imageHeight;
    node["camera_matrix"] >> cameraMat;
    node["distortion_coefficients"] >> distCoeffs;
    node["extrinsic_parameters"] >> extrinsicParam;
}
void readData_H(const FileStorage& node, Mat& H_L, Mat& H_R)
{
    node["H_LtoC"] >> H_L;
    node["H_RtoC"] >> H_R;
}
void readData_A(const FileStorage& node, Mat& alpha_L, Mat& alpha_R)
{
    node["alpha_left"] >> alpha_L;
    node["alpha_right"] >> alpha_R;
}
/*
 imshow("source", src);
 imshow("detected lines", cdst);
*/

 //waitKey();

 //return 0;
//}
