#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

#define YELLOW_HUE_LOWER 25
#define YELLOW_HUE_UPPER 50
#define BLUE_HUE_LOWER 100
#define BLUE_HUE_UPPER 110
#define GREEN_HUE_LOWER 80
#define GREEN_HUE_UPPER 90
#define BLUE 1
#define YELLOW 2
#define GREEN 3

Mat src; Mat src_hsv; Mat src_mask; Mat drawing; Mat canny_output;
int thresh = 100, ratio = 3;
int max_thresh = 255;
int area_thresh = 150;
RNG rng(12345);

void get_contours(int color, vector<int> &Cx, vector<int> &Cy)
{
  Scalar lower_limit, upper_limit;

  switch(color)
  {
  case BLUE:
    lower_limit = Scalar(BLUE_HUE_LOWER,0.85*255,0.50*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,255,255);
    break;
  case YELLOW:
    lower_limit = Scalar(YELLOW_HUE_LOWER,50,50);
    upper_limit = Scalar(YELLOW_HUE_UPPER,255,255);
    break;
  case GREEN:
    lower_limit = Scalar(GREEN_HUE_LOWER,0.75*255,0.45*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,255,0.65*255);
    break;
  }
  inRange(src_hsv,lower_limit,upper_limit,src_mask);
  
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
  morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

  /// Detect edges using canny
  Canny( src_mask, canny_output, thresh, thresh*ratio, 3 );

  /// Find contours
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  vector<int> filtered_contours_indices;
  cout<<"Number of contours found: "<< contours.size() <<endl;
  double area;
  for( size_t i = 0; i< contours.size(); i++ )
  {
    area = contourArea(contours[(int)i],true);
    if(area<area_thresh){
      cout<<area<<" ";
      continue;
    }
      
    Scalar color = Scalar( 255, 255, 255 );
    drawContours( drawing, contours, (int)i, color, 1, 8, hierarchy, 0, Point() );
    filtered_contours_indices.push_back((int)i);
  }
  int x, y;
  for( int i = 0; i < filtered_contours_indices.size(); i++ )
  {
    Moments m = moments(contours[filtered_contours_indices[i]]);
    x = m.m10/m.m00;
    y = m.m01/m.m00;
    Cx.push_back(x);
    Cy.push_back(y);
    cout<<" Filtered Area "<< m.m00 << " for color "<< color << " at "<< i <<endl;
    cout<<" (x,y): "<<x<<","<<y<<endl;
  }
  imshow( "Contours", drawing );
}

// void contour_thresh_cb(int, void*)
// {
//   Scalar lower_limit, upper_limit;
//   lower_limit = Scalar(GREEN_HUE_LOWER,0.75*255,0.45*255);
//   upper_limit = Scalar(GREEN_HUE_UPPER,255,0.65*255);

//   inRange(src_hsv,lower_limit,upper_limit,src_mask);
  
//   vector<vector<Point> > contours;
//   vector<Vec4i> hierarchy;

//   Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
//   morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

//   /// Detect edges using canny
//   Canny( src_mask, canny_output, thresh, thresh*ratio, 3 );

//   /// Find contours
//   findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
//   vector<int> filtered_contours_indices;
//   cout<<"Number of contours found: "<< contours.size() <<endl;
//   double area;
//   for( size_t i = 0; i< contours.size(); i++ )
//   {
//     area = contourArea(contours[(int)i]);
//     if(area<area_thresh){
//       cout<<area<<" ";
//       continue;
//     }
      
//     Scalar color = Scalar( 255, 255, 255 );
//     drawContours( drawing, contours, (int)i, color, 1, 8, hierarchy, 0, Point() );
//     filtered_contours_indices.push_back((int)i);
//   }
//   imshow( "Source", src_mask);
//   imshow( "Contours", canny_output );

// }

int main( int argc, char** argv )
{

  /// Create capture
  VideoCapture cap;
  cap.open(0);

  /// Create Window
  namedWindow( "Source", WINDOW_AUTOSIZE );
  namedWindow( "Contours", WINDOW_AUTOSIZE );

  src = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  
  /// Create a Trackbar for user to enter threshold
  // createTrackbar( "Min Threshold:", "Source", &thresh, max_thresh, contour_thresh_cb );

  vector<int> blue_cx, blue_cy, green_cx, green_cy, yellow_cx, yellow_cy;
  while(1)
  {
    /// Load source image and convert it to gray
    //cap >> src;

    /// Convert image to gray
    cvtColor(src, src_hsv, CV_BGR2HSV);
    drawing = Mat::zeros( src.size(), CV_8UC3 );
    get_contours(BLUE, blue_cx, blue_cy);
    get_contours(GREEN, green_cx, green_cy);
    get_contours(YELLOW, yellow_cx, yellow_cy);
    imshow( "Source", src);
    if(waitKey(10)>=0) break;
  }
  // waitKey(0);
  return(0);
}

