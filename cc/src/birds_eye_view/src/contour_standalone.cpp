#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
using namespace cv;
using namespace std;

#define YELLOW_HUE_LOWER 25
#define YELLOW_HUE_UPPER 50
#define BLUE_HUE_LOWER 95
#define BLUE_HUE_UPPER 125
#define GREEN_HUE_LOWER 70
#define GREEN_HUE_UPPER 95
#define BLUE 1
#define YELLOW 2
#define GREEN 3

Mat src; Mat src_hsv;
int area_thresh = 150;

void get_contours(int color, vector<Point> &C)
{
  Scalar lower_limit, upper_limit;
  Mat src_mask; Mat canny_output;

  switch(color)
  {
  case BLUE:
    lower_limit = Scalar(BLUE_HUE_LOWER,0.55*255,0.50*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,255,255);
    break;
  case YELLOW:
    lower_limit = Scalar(YELLOW_HUE_LOWER,50,50);
    upper_limit = Scalar(YELLOW_HUE_UPPER,255,255);
    break;
  case GREEN:
    lower_limit = Scalar(GREEN_HUE_LOWER,0.70*255,0.45*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,255,0.75*255);
    break;
  }
  inRange(src_hsv,lower_limit,upper_limit,src_mask);
  
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
  morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

  imshow( "Masks", src_mask );

  int thresh = 100, ratio = 3;
  int max_thresh = 255;

  /// Detect edges using canny
  Canny( src_mask, canny_output, thresh, thresh*ratio, 3 );

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
    C.push_back(Point(x,y));    
    // cout<<" Filtered Area "<< m.m00 << " for color "<< color << " at "<< i <<endl;
    // cout<<" (x,y): "<<x<<","<<y<<endl;
  }
  imshow( "Contours", drawing );
}

float dist(Point ptA, Point ptB)
{
  return sqrt((ptA.x-ptB.x)*(ptA.x-ptB.x)+(ptA.y-ptB.y)*(ptA.y-ptB.y));
}

int main( int argc, char** argv )
{

  /// Create capture
  VideoCapture cap(0);
  if(!cap.isOpened())
    return -  1;

  /// Create Window
  namedWindow( "Source", WINDOW_AUTOSIZE );
  namedWindow( "Masks", WINDOW_AUTOSIZE );
  namedWindow( "Contours", WINDOW_AUTOSIZE );
  int successful_frames = 0, skipped_frames = 0, total_frames=0;
  while(1)
  {
    /// Load source image and convert it to gray
    cap >> src;
    total_frames++;
    /// Convert image to gray
    cvtColor(src, src_hsv, CV_BGR2HSV); 
    
    vector<Point> blue_c, green_c, yellow_c;
    get_contours(YELLOW, yellow_c);
    get_contours(GREEN, green_c);
    get_contours(BLUE, blue_c);

    /// Extract robots from centroids
    // cout<< blue_cx.size()<<","<< blue_cy.size()<<","<< green_cx.size()<<","<< green_cy.size()<<","<< yellow_cx.size()<<","<< yellow_cy.size()<<endl;
    if(blue_c.size()==2&& green_c.size()==2&& yellow_c.size()==2){
      cout<<"Required contours found"<<endl;
      boost::numeric::ublas::vector<double> g (2), b (2), y(2);
      for (int i = 0; i < green_c.size(); ++i)
      {
        g(0) = green_c[i].x;
        g(1) = green_c[i].y;
        unsigned int min_dist =-1, min_j=0,min_k=0;
        for (int j = 0; j < yellow_c.size(); ++j)
        {
          if(dist(yellow_c[j],green_c[i])<min_dist){
            min_j = j;
            min_dist = dist(yellow_c[j],green_c[i]); 
          }
            
        }
        y(0) = yellow_c[min_j].x;
        y(1) = yellow_c[min_j].y;
        min_dist = -1;
        for (int k = 0; k < yellow_c.size(); ++k)
        {
          if(dist(blue_c[k],green_c[i])<min_dist){
            min_dist = dist(blue_c[k],green_c[i]);
            min_k = k;
          }
        }
        b(0) = blue_c[min_k].x;
        b(1) = blue_c[min_k].y;
        // line(src,green_c[i],yellow_c[min_j],Scalar(0,0,0),2,8);
        // line(src,yellow_c[min_j],blue_c[min_k],Scalar(0,0,0),2,8);
        // line(src,blue_c[min_k],green_c[i],Scalar(0,0,0),2,8);
        
        double  ip = inner_prod(b-g, y-g); 
        cout<<ip<<endl;
        boost::numeric::ublas::vector<double> ptA (2), ptB(2);
        if(ip>100){
          /// Robot 1
          ptA =  (4*b/5+g/5);
          ptB = ptA + (b-y);
          cout<<"Robot 1 at "<<ptA<<" and facing "<<ptB<<endl;
          line(src,Point(ptA(0),ptA(1)), Point(ptB(0),ptB(1)), Scalar(255,255,255), 1);
        }else{
          /// Robot 2
          ptA =  2*((b+y)/2)/3+g/3;
          ptB = ptA + (b-y);
          cout<<"Robot 1 at "<<ptA<<" and facing "<< ptB<<endl;
          line(src,Point(ptA(0),ptA(1)), Point(ptB(0),ptB(1)), Scalar(255,255,255), 1);
        }
        imshow( "Source", src);
      }
      successful_frames++;
    }else{
      cout<<"Not all contours were found, skipping to next frame"<<endl;
      skipped_frames++;
    }

    if(skipped_frames>successful_frames&&total_frames>1000)
    {
      cerr<<"Improper lighting. Exiting"<<endl;
      break;
    }

    if(waitKey(10)>=0) break;
  }
  // waitKey(0);
  return(0);
}

