#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

#define RED_HUE_LOWER 160
#define RED_HUE_UPPER 180
#define YELLOW_HUE_LOWER 25
#define YELLOW_HUE_UPPER 50
#define BLUE_HUE_LOWER 100
#define BLUE_HUE_UPPER 110
#define GREEN_HUE_LOWER 80
#define GREEN_HUE_UPPER 90

Mat src; Mat src_hsv; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

int hue_lower, hue_upper;

/// Function header
void thresh_callback(int, void* );

int main( int argc, char** argv )
{
  Scalar lower_limit, upper_limit;

  switch(atoi(argv[1]))
  {
  case 1:
    lower_limit = Scalar(RED_HUE_LOWER,0.10*255,0.45*255);
    upper_limit = Scalar(RED_HUE_UPPER,0.40*255,0.65*255);
    break;
  case 2:
    lower_limit = Scalar(BLUE_HUE_LOWER,0.85*255,0.50*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,255,255);
    break;
  case 3:
    lower_limit = Scalar(YELLOW_HUE_LOWER,50,50);
    upper_limit = Scalar(YELLOW_HUE_UPPER,255,255);
    break;
  case 4:
    lower_limit = Scalar(GREEN_HUE_LOWER,0.75*255,0.45*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,255,0.65*255);
    break;
  }
  /// Create capture
  VideoCapture cap;
  cap.open(0);

  /// Create Window
  const char* source_window = "Source";
  namedWindow( source_window, WINDOW_AUTOSIZE );

  namedWindow( "Contours", WINDOW_AUTOSIZE );

  while(1)
  {
    /// Load source image and convert it to gray
    cap >> src;

    /// Convert image to gray
    cvtColor(src, src_hsv, CV_BGR2HSV);
    inRange(src_hsv,lower_limit,upper_limit,src_hsv);
    // blur( src_hsv, src_hsv, Size(3,3) );
    // src_gray = src_hsv>128;
    imshow( source_window, src);

    
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using canny
    Canny( src_hsv, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
       {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
       }

    imshow( "Contours", drawing );
    if(waitKey(10)>=0) break;
  }
  return(0);
}

/**
 * @function thresh_callback
 */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_hsv, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }

  imshow( "Contours", drawing );
}