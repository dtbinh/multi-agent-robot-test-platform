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
#define BLUE_HUE_LOWER 95
#define BLUE_HUE_UPPER 125
#define GREEN_HUE_LOWER 70
#define GREEN_HUE_UPPER 95

Mat src; Mat src_hsv; Mat src_mask;
int thresh = 100;
int max_thresh = 255,area_thresh=100;
RNG rng(12345);

int hue_lower, hue_upper;

/// Function header
void onMouse(int event, int x, int y, int , void*)
{
  if(event != EVENT_LBUTTONDOWN)
    return;

  Vec3b hsv = src_hsv.at<Vec3b>(x,y);
  int h = hsv.val[0];
  int s = hsv.val[1];
  int v = hsv.val[2];
  cout<<h<<","<<s/2.55<<","<<v/2.55<<endl;
}

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
    lower_limit = Scalar(BLUE_HUE_LOWER,0.55*255,0.50*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,255,255);
    break;
  case 3:
    lower_limit = Scalar(YELLOW_HUE_LOWER,50,50);
    upper_limit = Scalar(YELLOW_HUE_UPPER,255,255);
    break;
  case 4:
    lower_limit = Scalar(GREEN_HUE_LOWER,0.70*255,0.45*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,255,0.75*255);
    break;
  }
  /// Create capture
  VideoCapture cap;
  cap.open(0);

  /// Create Window
  const char* source_window = "Source";
  namedWindow( source_window, WINDOW_AUTOSIZE );

  namedWindow( "Masks", WINDOW_AUTOSIZE );
  namedWindow( "Contours", WINDOW_AUTOSIZE );

  setMouseCallback("Masks", onMouse, 0);
  setMouseCallback("Source", onMouse, 0);
  
  while(1)
  {
    /// Load source image and convert it to gray
    cap >> src;
    imshow( source_window, src);
    /// Convert image to gray
    cvtColor(src, src_hsv, CV_BGR2HSV);
    inRange(src_hsv,lower_limit,upper_limit,src_mask);
    // blur( src_hsv, src_hsv, Size(3,3) );
    // src_gray = src_hsv>128;
    
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<int> Cx, Cy;

    Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
    morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

    imshow( "Masks", src_mask);

    /// Detect edges using canny
    Canny( src_mask, canny_output, thresh, thresh*3, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<int> filtered_contours_indices;
    // cout<<"Number of contours found: "<< contours.size() <<endl;
    double area;
    Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
      area = contourArea(contours[(int)i],true);
      if(area<area_thresh){
        // cout<<area<<" ";
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
      // cout<<" Filtered Area "<< m.m00 << " for color "<< atoi(argv[1]) << " at "<< i <<endl;
      // cout<<" (x,y): "<<x<<","<<y<<endl;
    }
    imshow( "Contours", drawing );
    if(waitKey(10)>=0) break;
  }
  return(0);
}
