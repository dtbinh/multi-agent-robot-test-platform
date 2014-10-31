#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblob.h>

using namespace std;
using namespace cv;
using namespace cvb;

#define FLOOR_HUE_LOWER 10
#define FLOOR_HUE_UPPER 20
#define SMILEY_HUE_LOWER 25
#define SMILEY_HUE_UPPER 40
#define BLUE_HUE_LOWER 90
#define BLUE_HUE_UPPER 130
#define RED_HUE_LOWER 0
#define RED_HUE_UPPER 10
#define YELLOW_HUE_LOWER 25
#define YELLOW_HUE_UPPER 50

int x[10] = {0}, y[10] = {0};

int main(){
	Mat frame;
	Mat frame_small;
	Mat frame_hsv;

	VideoCapture cap;
	cap.open(0);

	namedWindow("Captured Frame", CV_WINDOW_AUTOSIZE);
	namedWindow("Detected Blobs", CV_WINDOW_AUTOSIZE);

	while (1){
		cap >> frame;
		resize(frame,frame_small,Size(),1,1,CV_INTER_AREA);
		cvtColor(frame_small, frame_small, CV_BGR2HSV);
		inRange(frame_small,Scalar(YELLOW_HUE_LOWER,50,50),Scalar(YELLOW_HUE_UPPER,255,255),frame_hsv);
		//frame_hsv = 255-frame_hsv;
		//resize(image_hsv,image_hsv,Size(),4,4,CV_INTER_AREA);
		Mat frame_bw = frame_hsv>128;
		IplImage image_bw = frame_bw;
		IplImage image_small = frame_small;
		//cvThreshold(&hsv_1, &hsv_1, 100, 200, CV_THRESH_BINARY);
		IplImage *labelImg = cvCreateImage(cvGetSize(&image_bw), IPL_DEPTH_LABEL, 1);
		CvBlobs blobs;
		unsigned int result = cvLabel(&image_bw, labelImg, blobs);
		for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
		{
  			// cout << "Blob #" << it->second->label << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
  			x[it->second->label] = 0.8*x[it->second->label]+0.2*it->second->centroid.x;
  			y[it->second->label] = 0.8*x[it->second->label]+0.2*it->second->centroid.y;
		}
		cvRenderBlobs(labelImg, blobs, &image_small, &image_small);
		cvShowImage("Detected Blobs", &image_small);
		imshow("Captured Frame", frame);
		if(waitKey(10)>=0) break;
		cout << "(x,y) => (" << x[1] << "," << y[1] <<"); (x,y) => (" << x[2] << "," << y[2] << ")\n";
	}
	return 0;
}
