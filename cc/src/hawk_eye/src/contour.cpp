#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace cv;
using namespace std;

int BLUE_HUE_LOWER   = 98;
int BLUE_HUE_UPPER   = 120;
int YELLOW_HUE_LOWER = 25;
int YELLOW_HUE_UPPER = 70;
int GREEN_HUE_LOWER  = 85;
int GREEN_HUE_UPPER  = 97;

float BLUE_SAT_LOWER   = 0.80;
float BLUE_SAT_UPPER   = 1.00;
float YELLOW_SAT_LOWER = 0.25;
float YELLOW_SAT_UPPER = 1.00;
float GREEN_SAT_LOWER  = 0.95;
float GREEN_SAT_UPPER  = 1.00;

float BLUE_VALUE_LOWER   = 0.70;
float BLUE_VALUE_UPPER   = 1.00;
float YELLOW_VALUE_LOWER = 0.75;
float YELLOW_VALUE_UPPER = 1.00;
float GREEN_VALUE_LOWER  = 0.45;
float GREEN_VALUE_UPPER  = 0.95;

#define BLUE 1
#define YELLOW 2
#define GREEN 3

Mat src; Mat src_hsv;
int area_thresh = 50;

double theta_acc[3]={0,0,0};
float alpha = 1;

void get_contours(int color, vector<Point> &C)
{
  Scalar lower_limit, upper_limit;
  Mat src_mask; Mat canny_output;

  switch(color)
  {
  case BLUE:
    lower_limit = Scalar(BLUE_HUE_LOWER,BLUE_SAT_LOWER*255,BLUE_VALUE_LOWER*255);
    upper_limit = Scalar(BLUE_HUE_UPPER,BLUE_SAT_UPPER*255,BLUE_VALUE_UPPER*255);
    break;
  case YELLOW:
    lower_limit = Scalar(YELLOW_HUE_LOWER,YELLOW_SAT_LOWER*255,YELLOW_VALUE_LOWER*255);
    upper_limit = Scalar(YELLOW_HUE_UPPER,YELLOW_SAT_UPPER*255,YELLOW_VALUE_UPPER*255);
    break;
  case GREEN:
    lower_limit = Scalar(GREEN_HUE_LOWER,GREEN_SAT_LOWER*255,GREEN_VALUE_LOWER*255);
    upper_limit = Scalar(GREEN_HUE_UPPER,GREEN_SAT_UPPER*255,GREEN_VALUE_UPPER*255);
    break;
  }
  inRange(src_hsv,lower_limit,upper_limit,src_mask);
  
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  Mat st = getStructuringElement(MORPH_CROSS, Size(3,3));
  morphologyEx(src_mask, src_mask, MORPH_OPEN, st);

  // imshow( "Masks", src_mask );

  int thresh = 50, ratio = 3;
  int max_thresh = 255;

  /// Detect edges using canny
  blur( src_mask, src_mask, Size(3,3) );
  Canny( src_mask, canny_output, thresh, thresh*ratio, 3 );
  //imshow("Contours", canny_output);
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0) );
  vector<int> filtered_contours_indices;
  // cout<<"Number of contours found: "<< contours.size() <<endl;
  double area;
  Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
    area = contourArea(contours[(int)i],false);
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
  // imshow( "Contours", drawing );
}

float dist(Point ptA, Point ptB)
{
  return sqrt((ptA.x-ptB.x)*(ptA.x-ptB.x)+(ptA.y-ptB.y)*(ptA.y-ptB.y));
}

int main( int argc, char** argv )
{

  ros::init(argc, argv, "hawk_eye");
  ros::NodeHandle n;
  ros::Publisher pose_pub[3];
  pose_pub[0] = n.advertise<geometry_msgs::PoseStamped>("/robot1/pose_estimate", 1000);
  pose_pub[1] = n.advertise<geometry_msgs::PoseStamped>("/robot2/pose_estimate", 1000);
  pose_pub[2] = n.advertise<geometry_msgs::PoseStamped>("/robot3/pose_estimate", 1000);

  if(! ros::param::get("/hsv_thresholds/blue_hue_lower", BLUE_HUE_LOWER)){
    BLUE_HUE_LOWER = 98;
  }
  if(! ros::param::get("/hsv_thresholds/blue_hue_upper", BLUE_HUE_UPPER)){
    BLUE_HUE_UPPER   = 120;    
  }
  if(! ros::param::get("/hsv_thresholds/yellow_hue_lower",YELLOW_HUE_LOWER)){
    YELLOW_HUE_LOWER = 25;
  }
  if(! ros::param::get("/hsv_thresholds/yellow_hue_upper",YELLOW_HUE_UPPER)){
    YELLOW_HUE_UPPER = 70;
  }
  if(! ros::param::get("/hsv_thresholds/green_hue_lower",GREEN_HUE_LOWER)){
    GREEN_HUE_LOWER  = 85;
  }
  if(! ros::param::get("/hsv_thresholds/green_hue_upper",GREEN_HUE_UPPER)){
    GREEN_HUE_UPPER  = 97;
  }
  if(! ros::param::get("/hsv_thresholds/yellow_sat_lower",YELLOW_SAT_LOWER)){
    YELLOW_SAT_LOWER  = 0.25;
  }
  if(! ros::param::get("/hsv_thresholds/yellow_sat_upper",YELLOW_SAT_UPPER)){
    YELLOW_SAT_UPPER  = 1.00;
  }
  if(! ros::param::get("/hsv_thresholds/green_sat_lower",GREEN_SAT_LOWER)){
    GREEN_SAT_LOWER  = 0.90;
  }
  if(! ros::param::get("/hsv_thresholds/green_sat_upper",GREEN_SAT_UPPER)){
    GREEN_SAT_UPPER  = 1.00;
  }
  if(! ros::param::get("/hsv_thresholds/blue_sat_lower",BLUE_SAT_LOWER)){
    BLUE_SAT_LOWER  = 0.80;
  }
  if(! ros::param::get("/hsv_thresholds/blue_sat_upper",BLUE_SAT_UPPER)){
    BLUE_SAT_UPPER  = 1.00;
  }
  if(! ros::param::get("/hsv_thresholds/green_value_lower",GREEN_VALUE_LOWER)){
    GREEN_VALUE_LOWER  = 0.45;
  }
  if(! ros::param::get("/hsv_thresholds/green_value_upper",GREEN_VALUE_UPPER)){
    GREEN_VALUE_UPPER  = 0.95;
  }
  if(! ros::param::get("/hsv_thresholds/yellow_value_lower",YELLOW_VALUE_LOWER)){
    YELLOW_VALUE_LOWER  = 0.75;
  }
  if(! ros::param::get("/hsv_thresholds/yellow_value_upper",YELLOW_VALUE_UPPER)){
    YELLOW_VALUE_UPPER  = 1.00;
  }
  if(! ros::param::get("/hsv_thresholds/blue_value_lower",BLUE_VALUE_LOWER)){
    BLUE_VALUE_LOWER  = 0.70;
  }
  if(! ros::param::get("/hsv_thresholds/blue_value_upper",BLUE_VALUE_UPPER)){
    BLUE_VALUE_UPPER  = 1.00;
  }
  cout<<GREEN_HUE_LOWER<<","<<GREEN_HUE_UPPER<<endl;
  cout<<BLUE_HUE_LOWER<<","<<BLUE_HUE_UPPER<<endl;
  cout<<YELLOW_HUE_LOWER<<","<<YELLOW_HUE_UPPER<<endl;
  cout<<GREEN_SAT_LOWER<<","<<GREEN_SAT_UPPER<<endl;
  cout<<BLUE_SAT_LOWER<<","<<BLUE_SAT_UPPER<<endl;
  cout<<YELLOW_SAT_LOWER<<","<<YELLOW_SAT_UPPER<<endl;
  cout<<GREEN_VALUE_LOWER<<","<<GREEN_VALUE_UPPER<<endl;
  cout<<BLUE_VALUE_LOWER<<","<<BLUE_VALUE_UPPER<<endl;
  cout<<YELLOW_VALUE_LOWER<<","<<YELLOW_VALUE_UPPER<<endl;

  /// Create capture
  VideoCapture cap(0);
  if(!cap.isOpened())
    return -1;

  /// Create Window
  // namedWindow( "Source", WINDOW_AUTOSIZE );
  // namedWindow( "Masks", WINDOW_AUTOSIZE );
  // namedWindow( "Contours", WINDOW_AUTOSIZE );
  int successful_frames = 0, skipped_frames = 0, total_frames=0;
  ros::Rate loop_rate(1);
  while(ros::ok())
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
          ROS_WARN("Nearby yellow not found. Skipping to next pattern.");
          continue;
        }
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
          ROS_WARN("Nearby blue not found. Skipping to next pattern.");
          continue;
        }
        // line(src,green_c[i],yellow_c[min_j],Scalar(0,0,0),2,8);
        // line(src,yellow_c[min_j],blue_c[min_k],Scalar(0,0,0),2,8);
        // line(src,blue_c[min_k],green_c[i],Scalar(0,0,0),2,8);
        double A = norm_2(b-y), B = norm_2(y-g), C = norm_2(g-b);
        double  ip = acos((B*B+C*C-A*A)/(2*B*C)), theta;
        int robot_no = 0;
        // cout<<ip<<endl;
        boost::numeric::ublas::vector<double> ptA (2), ptB(2), x_unit (2);
        x_unit(0) = 0;
        x_unit(1) = 1;
        if(ip>1.65){
          /// Robot 1
          ptA = (b+y)/2;
          ptB = g;
          robot_no = 1;
        }else if(ip>1.0){
          /// Robot 2
          ptA = (8.595*b+4.785*y)/13.38;
          ptB = g;
          robot_no = 2;
        }else{
          /// Robot 3
          ptA = (8.595*b-4.785*y)/3.81;
          ptB = g;
          robot_no = 3;
        }
        theta = acos(inner_prod((ptB-ptA)/sqrt(inner_prod(ptB-ptA,ptB-ptA)), x_unit));
        if((ptB-ptA)(0)<0)
          theta = -theta;
        theta = alpha*theta + (1-alpha)*theta_acc[robot_no-1];
        theta_acc[robot_no-1] = theta;
        // conic_projection(&ptA);
        ROS_INFO("%f Robot %d at %f,%f and facing %f", ip, robot_no, ptA(1), ptA(0), theta*180/M_PI);
        line(src,Point(ptA(0),ptA(1)), Point(ptB(0),ptB(1)), Scalar(255,255,255), 2);
        imshow("Source",src);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id="map";
        geometry_msgs::Pose pose;
        pose.position.x = ptA(1)*1.77/640;
        pose.position.y = ptA(0)*1.77/640;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
        pose.orientation = quat;
        pose_stamped.pose = pose;
        pose_pub[robot_no-1].publish(pose_stamped);
      }
      successful_frames++;
    }else{
      ROS_WARN("Not all contours were found, skipping to next frame. Green: %d Yellow: %d Blue: %d", (int)green_c.size(), (int)yellow_c.size(), (int)blue_c.size());
      skipped_frames++;
    }

    if(skipped_frames>successful_frames&&total_frames>1000)
    {
      ROS_ERROR("Improper lighting. Exiting");
      return -1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  // waitKey(0);
  return(0);
}

