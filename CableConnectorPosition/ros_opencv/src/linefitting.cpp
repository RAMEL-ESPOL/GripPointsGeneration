#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <tuple>
#define PI 3.141592653
#define ABS(a)      (((a)<(0))?-(a):(a))
//#define MAX(a,b)    (((a)>(b))?(a):(b)) // already defined by openCV
#define Deg2Rad 0.01745329251 // pi/180
#define Rad2Deg 57.295779546 // 180/pi
#define MAX_RPM 300

using namespace cv;
using namespace std;

std::tuple<float, float, float, float>Quaternion(float fYaw)
{
	float fRoll = 0.0;
	float fPitch = 0.0;
	fYaw *= Deg2Rad;
	const float fSinPitch(sin(fPitch*0.5F));
	const float fCosPitch(cos(fPitch*0.5F));
	const float fSinYaw(sin(fYaw*0.5F));
	const float fCosYaw(cos(fYaw*0.5F));
	const float fSinRoll(sin(fRoll*0.5F));
	const float fCosRoll(cos(fRoll*0.5F));
	const float fCosPitchCosYaw(fCosPitch*fCosYaw);
	const float fSinPitchSinYaw(fSinPitch*fSinYaw);
	float X = fSinRoll * fCosPitchCosYaw - fCosRoll * fSinPitchSinYaw;
	float Y = fCosRoll * fSinPitch * fCosYaw + fSinRoll * fCosPitch * fSinYaw;
	float Z = fCosRoll * fCosPitch * fSinYaw - fSinRoll * fSinPitch * fCosYaw;
	float W = fCosRoll * fCosPitchCosYaw + fSinRoll * fSinPitchSinYaw;

    return {W, X, Y, Z};

}

struct myclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x > pt2.x);}
} object_x;

struct myclass2 {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
} object_y;

int main(int argc, char** argv)
{
  float cm_to_pixel = 32.8/640;

    String filename("/home/francisco/robotis_ws/src/ros-bioloid-vision/ros_opencv/src/171.jpg");

    Mat src = imread(filename, 0);
    
    cout << src << endl;
    
 if(src.empty())
 {
     cout << "can not open " << filename << endl;
     cv::waitKey(5000);
     return -1;
 }

 cv::imshow("Original", src);

 Mat dst, result;
 Canny(src, dst, 50, 150, 3);
 cvtColor(dst, result, CV_GRAY2BGR);

   //Rotate an Image
    double angle = -90;

    // get rotation matrix for rotating the image around its center
    cv::Point2f center(result.cols/2.0, result.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle
    cv::Rect bbox = cv::RotatedRect(center,result.size(), angle).boundingRect();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;

    cv::Mat rot_dst;
    cv::warpAffine(result, rot_dst, rot, bbox.size());
    cv::imwrite("/home/francisco/robotis_ws/src/ros-bioloid-vision/ros_opencv/src/rotated_im.png", rot_dst);

/**
 * Resize mat
 */
const int kNewWidth = 640;
const int kNewHeight = 480;

Mat frame;
resize(rot_dst, frame, cvSize(kNewWidth, kNewHeight));
cv::imwrite("/home/francisco/robotis_ws/src/ros-bioloid-vision/ros_opencv/src/resized_im.png", frame);
/* //Resize IplImage

IplImage* img; // Input
IplImage* new_img = cvCreateImage(cvSize(kNewWidth, kNewHeight), img->depth, img->nChannels);
cvResize(img, new_img);*/

result = rot_dst.clone();
cout << "Width : " << result.size().width << endl;
cout << "Height: " << result.size().height << endl;


 //srand(time(NULL)) ;

	//-------------------------------------------------------------- make sample data
	/* random number in range 0 - 1 not including 1 */
	//float random = 0.f;
	/* the white noise */
	//float noise = 0.f;
	/* Setup constants */
	//const static int q = 15;
	//const static float c1 = (1 << q) - 1;
	//const static float c2 = ((int)(c1 / 3)) + 1;
	//const static float c3 = 1.f / c1;

	double noise_sigma = 100 ;
	///double x[100] ; // x - bizim row
	///double y[100] ; // y - bu ise column of canny result olacaq

	std::vector<double> vec_x;
    std::vector<double> vec_y;
    for (int y=0; y<result.rows; y++)
    {
        Vec3b* res = result.ptr<Vec3b>(y);
        for (int x=0; x<result.cols; x++)// x=320
        {
            if (((int)res[x][0])!=0)
            {
                vec_x.push_back(x);
                vec_y.push_back(y);
                //poi.push_back(Point(x,y));
            }

        }
    }
    int sample_size = vec_x.size();


  /*for (int i=0;i<sample_size;i++)
  {
      //std::cout << sample_size << "aa" << i << std::endl;
      std::cout << vec_x[i] << std::endl;


  }*/
  /*for (std::vector<double>::const_iterator t = vec_x.begin(); t != vec_x.end(); ++t){

    std::cout << *t << ' ';
  }
*/
/*
	double iA = 0.005 ;
	double iB = 0.5 ;
	double iC = 0 ;
	*for( int i=0 ; i<100 ; i++ )
	{
		x[i] = i ;
		y[i] = iA*(vec_x[i]*vec_x[i]) + iB*vec_x[i] + iC ; // instead of use sizeof func canny result

#if 0
		if( i > 50 && i < 70 )
		{
			y[i] += 0.1*abs(x[i]);
		}
#endifstd::cout << "b---------------------" << std::endl;
		//y[i] = y[i] + noise_sigma*((rand()%100)+1) ;

		random = ((float)rand() / (float)(RAND_MAX + 1));
		noise = (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;

		int noise_scale = 2.0 ;
		if( i > 50 && i<70 ) noise_scale = 5.0 ;
		y[i] = y[i] + noise*noise_scale ;
	}*/

	//-------------------------------------------------------------- build matrix
	cv::Mat A(sample_size, 3, CV_64FC1) ; /// Ax^2+ Bx+ C vec_x
	cv::Mat B(sample_size,1, CV_64FC1) ; ///  y

	for( int i=0 ; i<sample_size ; i++ ) // sizeof instead all 100
	{
		A.at<double>(i,0) = vec_x[i] * vec_x[i] ;
		A.at<double>(i,1) = vec_x[i] ;
		A.at<double>(i,2) = 1.0 ;
	}
	for( int i=0 ; i<sample_size; i++ )
	{
		B.at<double>(i,0) = vec_y[i];
	}

	//-------------------------------------------------------------- RANSAC fitting
	//int n_data = 100 ;
	int N = 100;	//iterations
	double T = 3*noise_sigma;   // residual threshold

	//int n_sample = 3;
	int max_cnt = 0;

	cv::Mat best_model(3,1,CV_64FC1) ;

	for( int i=0 ; i<N ; i++ )
	{
		//random sampling - 3 point
		int k[3] = {-1, } ;
		k[0] = floor((rand()%sample_size+1))+1; //just random number.Anything can be

		do
		{
			k[1] = floor((rand()%sample_size+1))+1;
		}while(k[1]==k[0] || k[1]<0) ;

		do
		{
			k[2] = floor((rand()%sample_size+1))+1;
		}while(k[2]==k[0] || k[2]==k[1] || k[2]<0) ;

		//printf("random sample : %d %d %d\n", k[0], k[1], k[2]) ;

		//model estimation
		cv::Mat AA(3,3,CV_64FC1) ; // y' = k*y= k(Ax+ BX +c) modelini burda duzeltdi
		cv::Mat BB(3,1, CV_64FC1) ;
		for( int j=0 ; j<3 ; j++ )
		{
			AA.at<double>(j,0) = vec_x[k[j]] * vec_x[k[j]] ; // take 3 points from first column of A matrix
			AA.at<double>(j,1) = vec_x[k[j]] ; // take 3 points from 2nd column of A matrix
			AA.at<double>(j,2) = 1.0 ; // take 3 points from 3rd column of A matrix

			BB.at<double>(j,0) = vec_y[k[j]] ; // take 3 points from B matrix
		}

		cv::Mat AA_pinv(3,3,CV_64FC1) ;
		invert(AA, AA_pinv, cv::DECOMP_SVD); // 1/AA

		cv::Mat X = AA_pinv * BB ; // X = 1/AA * BB
        //x[i] = i ;
		//y[i] = iA*(x[i]*x[i]) + iB*x[i] + iC ;

		//evaluation
		cv::Mat residual(100,1,CV_64FC1) ; // sizeof canny edge result
		residual = cv::abs(B-A*X) ; // residual = y -y' = y - k(Ax+Bx+C)
		int cnt = 0 ;
		for( int j=0 ; j<100 ; j++ )
		{
			double data = residual.at<double>(j,0) ;

			if( data < T )
			{
				cnt++ ;
			}
		}

		if( cnt > max_cnt )
		{
			best_model = X ;
			max_cnt = cnt ;
		}
	}

	//------------------------------------------------------------------- optional LS fitting
	cv::Mat residual = cv::abs(A*best_model - B) ;
	std::vector<int> vec_index ;
	for( int i=0 ; i<sample_size; i++ )
	{
		double data = residual.at<double>(i, 0) ;
		if( data < T )
		{
			vec_index.push_back(i) ;
		}
	}

	cv::Mat A2(vec_index.size(),3, CV_64FC1) ;
	cv::Mat B2(vec_index.size(),1, CV_64FC1) ;

	for( size_t i=0 ; i<vec_index.size(); i++ )
	{
		A2.at<double>(i,0) = vec_x[vec_index[i]] * vec_x[vec_index[i]]  ;
		A2.at<double>(i,1) = vec_x[vec_index[i]] ;
		A2.at<double>(i,2) = 1.0 ;

		B2.at<double>(i,0) = vec_y[vec_index[i]] ;
	}

	cv::Mat A2_pinv(3,vec_index.size(),CV_64FC1) ;
	invert(A2, A2_pinv, cv::DECOMP_SVD);

	cv::Mat X = A2_pinv * B2 ;

	//Drawing
	cv::Mat F = A*X ;
	printf("matrix F : cols =%d, rows=%d\n", F.cols, F.rows) ;
	std::cout << X << " " <<X.type() << std::endl;
	std::cout << X.at<double>(0,0)<< " "<< X.at<double>(1,0) << std::endl;


	int interval = 1 ;
	//cv::Mat imgResult(100*interval,100*interval,CV_8UC3) ;
	//imgResult = cv::Scalar(0) ;
	for( int iy=0 ; iy<sample_size ; iy++ )
	{
		cv::circle(result, cv::Point(vec_x[iy]*interval, vec_y[iy]*interval) ,3, cv::Scalar(0,0,255), CV_FILLED) ;

		double data = F.at<double>(iy,0) ;

		cv::circle(result, cv::Point(vec_x[iy]*interval, data*interval) ,2, cv::Scalar(0,255,0), CV_FILLED) ;
	}
	//cv::imshow("Not_Rotated------", result) ;

	std::vector<double> vec_xx;
    std::vector<double> vec_yy;

    for (int isx=0; isx<result.rows; isx++)
    {
        Vec3b* res2 = result.ptr<Vec3b>(isx);
        for (int isy=0; isy<result.cols; isy++)// x=320
        {
            if (((int)res2[isy][1])==255)
            {
                vec_xx.push_back(isx);
                vec_yy.push_back(isy);
            }

        }
    }
   std::vector<cv::Point> pts(vec_yy.size()-40);
   for (int i = 0; i < vec_yy.size()-40; i++)
   {
       pts[i] = Point(vec_yy[i],vec_xx[i]);
   }
   std::sort(pts.begin(), pts.end(),object_x);
   int pt_num =20;
   int div=(int)pts.size()/pt_num;
   printf("%d %d", pts.size(), div);
   std::vector<cv::Point> pts_extracted;
   std::vector<cv::Point> pts_subtr(20);
   for (int i = 0; i < pt_num; i++)
   {
     pts_extracted.push_back(pts[i*div]);
   }

   for (int i = 0; i < pts_extracted.size(); i++)
   {
      printf("x_cm=%d, y_cm=%d \n", pts_extracted[i].x, pts_extracted[i].y);
   }


 //Convert to Quaternion
   for (int i = 0; i < pts_extracted.size(); i++){
        double gradient = 2*X.at<double>(0,0)*pts_extracted[i].x + X.at<double>(1,0);
        float radian = atan2(gradient, 1);
        float angle = (radian) * 180 / PI;
       // std::cout << angle << std::endl;
        //angle = Yaw value (in Degree)
        auto [Q1, Q2, Q3, Q4] = Quaternion(angle);
        cout << "Angle:" << angle<<"  "<< "Quaternion: " << Q1 << ','<< Q2 << ','<< Q3 << ','<< Q4 << endl;
   }


//////////////////////////////////
//Rotate the Image
    double angle_s = 90;

    // get rotation matrix for rotating the image around its center
    cv::Point2f center2(result.cols/2.0, result.rows/2.0);
    cv::Mat rot2 = cv::getRotationMatrix2D(center2, angle_s, 1.0);
    // determine bounding rectangle
    cv::Rect bbbox = cv::RotatedRect(center2,result.size(), angle_s).boundingRect();
    // adjust transformation matrix
    rot2.at<double>(0,2) += bbbox.width/2.0 - center2.x;
    rot2.at<double>(1,2) += bbbox.height/2.0 - center2.y;

    cv::Mat rot_dst_s;
    cv::warpAffine(result, rot_dst_s, rot2, bbbox.size());
    cv::imwrite("/home/francisco/robotis_ws/src/ros-bioloid-vision/ros_opencv/src/Rotated.png", rot_dst_s);
    imshow("Rotated", rot_dst_s);

    //const float xScaleFactor = 1.33;
	//const float yScaleFactor = 0.74852;

	//Mat scaled;
    //resize(rot_dst_s, scaled, cvSize(0, 0),xScaleFactor, yScaleFactor);
    //imshow("Scaled", scaled);

     result = rot_dst_s.clone();
     std::vector<double> vec_xxx;
     std::vector<double> vec_yyy;

    for (int isx=0; isx<result.rows; isx++)
    {
        Vec3b* res3 = result.ptr<Vec3b>(isx);
        for (int isy=0; isy<result.cols; isy++)// x=320
        {
            if (((int)res3[isy][1])==255)
            {
                vec_xxx.push_back(isx);
                vec_yyy.push_back(isy);
            }

        }
    }
   std::vector<cv::Point> pts2(vec_yyy.size()-40);
   for (int i = 0; i < vec_yyy.size()-40; i++)
   {
       pts2[i] = Point(vec_yyy[i],vec_xxx[i]);
   }
   std::sort(pts2.begin(), pts2.end(),object_y);
   int div2=(int)pts2.size()/pt_num;
   printf("%d %d", pts2.size(), div2);
   std::vector<cv::Point> pts_extracted2;
   std::vector<cv::Point> pts_subtr2(20);
   for (int i = 0; i < pt_num; i++)
   {
     pts_extracted2.push_back(pts2[i*div2]);
   }

   /*for (int i = 0; i < pts2.size(); i++)
   {
      printf("x=%d, y=%d \n", pts2[i].x, pts2[i].y);
   }*/
   printf("\n");

   for (int i = 0; i < pts_extracted2.size(); i++)
   {
      printf("x_cm=%d, y_cm=%d \n", pts_extracted2[i].x, pts_extracted2[i].y);
   }

    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(0),   -sin(0),
               0,       sin(0),   cos(0)
               );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(0),    0,      sin(0),
               0,               1,      0,
               -sin(0),   0,      cos(0)
               );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(PI),    -sin(PI),      0,
               sin(PI),    cos(PI),       0,
               0,               0,                  1);

    // Combined rotation matrix
    Mat Ro_c = R_z;
    //std::cout<< Ro_c << std::endl;
    Mat do_c = (Mat_<double>(1,3) << 16.3, -1, 0);
    std::cout<< do_c << std::endl;
    std::cout<< "check Ro_c\n" << Ro_c << std::endl;

    Mat Ho_c = Mat::zeros(4,4, Ro_c.type());
    Ro_c.copyTo(Ho_c(Range(0,3),Range(0,3)));
    Ho_c(Range(0,3),Range(3,4))= do_c.t();
    Ho_c.at<double>(3,3) = 1.0;
    std::cout<< "check Ho_c \n" << Ho_c << std::endl;
    //Mat PC = Mat::zeros(pts_extracted2.size(),4, Ho_c.type());
   /* for (int i = 0; i < pts_extracted2.size(); i++)
        {
          PC = (Mat_<float>(pts_extracted2.size(),4) << pts_extracted2[i].x,pts_extracted2[i].y, 0, 1);
        }*/
#if 1
    Mat PC = cv::Mat::zeros(4, pts_extracted2.size(), CV_64F);
    for (int i=0; i<pts_extracted2.size(); i++){
        PC.at<double>(0,i) = pts_extracted2[i].x*cm_to_pixel;
        PC.at<double>(1,i) = pts_extracted2[i].y*cm_to_pixel;
        PC.at<double>(2,i) = 0.;
        PC.at<double>(3,i) = 1.;
    }
    Mat PO = Ho_c * PC;
    std::cout<< "check PO \n" << PO.t() << std::endl;
#else
     for (int i=0; i<pts_extracted2.size(); i++){
        Mat PC = (Mat_<double>(4,1) << pts_extracted2[i].x*cm_to_pixel,pts_extracted2[i].y*cm_to_pixel, 0, 1);
        Mat PO = Ho_c * PC;
        //std::cout<< "check PC " << PC << std::endl;
        std::cout<< "check PO \n" << PO << std::endl;
     }
#endif // 1


    //Mat Ho_c = hconcat((Ro_c, do_c),1);
    //Ho_c = hconcat((Ho_c,[[0,0,0,1]]),0);


/*
    std::vector<Point> binVec;
    for (int isx = 0; isx < result.rows; isx++)
	{
		for ( int isy = 0; isy < result.cols; isy++)
		{
		    if(result.at<Vec3b>(isx, isy)[1]== 255)
                {
                   std::cout<<"x="<<isx<<"\t y="<<isy<<"\n";
                   binVec.push_back(Point(isx,isy));
                   //printf("Points : x= %d y= %d\n", binVec[isx], binVec[isy]);
                }
                else if(result.at<Vec3b>(isx, isy)[2]== 255)
                {
                    result.at<Vec3b>(isx, isy)[2]=0;
                }
		}
	}
*/


	cv::waitKey(0) ;

	return 0 ;
}