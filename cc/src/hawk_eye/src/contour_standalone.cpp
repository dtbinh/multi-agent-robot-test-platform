#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace cv;
using namespace std;

#define YELLOW_HUE_LOWER 25
#define YELLOW_HUE_UPPER 70
#define BLUE_HUE_LOWER 98
#define BLUE_HUE_UPPER 110
#define GREEN_HUE_LOWER 85
#define GREEN_HUE_UPPER 97
#define BLUE 1
#define YELLOW 2
#define GREEN 3

Mat src; Mat src_hsv;
int area_thresh = 50;

void get_contours(int color, vector<Point> &C)
{
  Scalar lower_limit, upper_limit;
  Mat src_mask; Mat canny_output;

  switch(color)
  {
  case BLUE:
    lower_limit = Scalar(BLUE_HUE_LOWER,0.80*255,0.70*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,255,255);
    break;
  case YELLOW:
    lower_limit = Scalar(YELLOW_HUE_LOWER,0.25*255,0.85*255);
    upper_limit = Scalar(YELLOW_HUE_UPPER,255,255);
    break;
  case GREEN:
    lower_limit = Scalar(GREEN_HUE_LOWER,0.95*255,0.35*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,255,0.95*255);
  }
  inRange(src_hsv,lower_limit,upper_limit,src_mask);
  
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
  morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

  imshow( "Masks", src_mask );

  int thresh = 50, ratio = 3;
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
      
    Scalar color = Scalar( 0, 0, 255 );
    drawContours( src, contours, (int)i, color, 1, 8, hierarchy, 0, Point() );
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
  // imshow( "Contours", drawing );
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
  // namedWindow( "Contours", WINDOW_AUTOSIZE );
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
    get_contours(BLUE, blue_c);
    get_contours(GREEN, green_c);

    /// Extract robots from centroids
    // cout<< blue_cx.size()<<","<< blue_cy.size()<<","<< green_cx.size()<<","<< green_cy.size()<<","<< yellow_cx.size()<<","<< yellow_cy.size()<<endl;
    if(blue_c.size()>0&& green_c.size()>0&& yellow_c.size()>0){
      // cout<<"Required contours found"<<endl;
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
        if(min_dist>30){
          cout<<"Nearby yellow not found. Skipping to next pattern."<<endl;
          continue;
        }
        cout<<"Min distances :"<<min_dist;
        min_dist = -1;
        for (int k = 0; k < blue_c.size(); ++k)
        {
          if(dist(blue_c[k],green_c[i])<min_dist){
            min_dist = dist(blue_c[k],green_c[i]);
            min_k = k;
          }
        }
        b(0) = blue_c[min_k].x;
        b(1) = blue_c[min_k].y;
        if(min_dist>30){
          cout<<"Nearby blue not found. Skipping to next pattern."<<endl;
          continue;
        }
        cout<<", "<<min_dist<<endl;
        // line(src,green_c[i],yellow_c[min_j],Scalar(0,0,0),2,8);
        // line(src,yellow_c[min_j],blue_c[min_k],Scalar(0,0,0),2,8);
        // line(src,blue_c[min_k],green_c[i],Scalar(0,0,0),2,8);
        
        double  ip = inner_prod(b-g, y-g), theta = 0; 
        cout<<ip<<endl;
        boost::numeric::ublas::vector<double> ptA (2), ptB(2), x_unit(2);
        x_unit(0) = 0;
        x_unit(1) = 1;
        // theta = acos(inner_prod((b-y)/sqrt(inner_prod(b-y,b-y)), x_unit))+M_PI/2;
        // if((b-y)(0)<0)
          // theta = -acos(inner_prod((b-y)/sqrt(inner_prod(b-y,b-y)), x_unit))+M_PI/2;
        if(ip>100){
          /// Robot 1
          ptA = (8.595*b-4.785*y)/3.81;
          ptB = g;
          cout<<"Robot 1 at ";
        }else if(ip>0){
          /// Robot 2
          ptA = (8.595*b+4.785*y)/13.38;
          ptB = g;
          cout<<"Robot 2 at ";
        }else{
          /// Robot 3
          ptA = (b+y)/2;
          ptB = g;
          cout<<"Robot 3 at ";
          
        }
        theta = acos(inner_prod((ptB-ptA)/sqrt(inner_prod(ptB-ptA,ptB-ptA)), x_unit));
        if((ptB-ptA)(0)<0)
          theta = -theta;
        cout<<ptA(1)*1.77/640<<","<<ptA(0)*1.77/640<<" and facing "<< theta*180/M_PI<<endl;
        line(src,Point(ptA(0),ptA(1)), Point(ptB(0),ptB(1)), Scalar(255,255,255), 2);
        imshow("Source",src);
      }
      successful_frames++;
    }else{
      cout<<"Not all contours were found, skipping to next frame"<<endl;
      cout<<"Number of contours: "<<green_c.size()<<","<<yellow_c.size()<<","<<blue_c.size()<<endl;
      skipped_frames++;
    }
    if(skipped_frames>successful_frames&&total_frames>1000)
    {
      cerr<<"Improper lighting. Exiting"<<endl;
      break;
    }

    if(waitKey(100)>=0) break;
  }
  // waitKey(0);
  return(0);
}