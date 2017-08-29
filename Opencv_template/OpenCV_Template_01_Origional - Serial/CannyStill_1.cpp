// CannyWebcam.cpp

#define COM_PORT "COM5"

#include"stdafx.h"
#using <System.dll>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<io.h>
#using <mscorlib.dll>
using namespace System;
using namespace System::IO::Ports;

#define WINDOW_W 640
#define WINDOW_H 480

//----- Forward Declarations -----
void SendByte(int toSend);
cv::Vec3f GetValsAtRoi(int &x1, int &y1, int &width, int &height, cv::Mat &frame);
void ManualCalibrate(int* hueLow, int* hueHigh, int* satLow, int* satHigh, int* valLow, int* valHigh);
void AutoCalibrate(int *hueLow, int *hueHigh, int *satLow, int *satHigh, int *valLow, int *valHigh, int &x1, int &y1, int &width, int &height, cv::Mat &frame);
void MouseMoveCallback(int event, int x, int y, int flags, void *userdata);
void MouseLClickCallback(int event, int x, int y, int flags, void *userdata);
void PerformPerspectiveTransform(cv::Mat &frameIn, cv::Mat &frameOut, cv::Point &size, cv::Point2f actual[], cv::Point2f desired[]);
cv::Point mousePose = cv::Point(200, 200); // I know this is naughty, but I can't get the bloody callback function to update the pointer!!!

//debugging
bool WEBCAM = false;
bool SAVE_IMAGES = false;
const std::string FILE_NAME = "C:/Users/Josh/Desktop/Images/frame_";
#define WEB_CAM 0 // (0) for integrated webcam (1) for USB webcma
#define REF_RECT 
const cv::Point2f RECT_LT  = cv::Point2f(0, 0);
const cv::Point2f RECT_LR  = cv::Point2f(250, 0);
const cv::Point2f  RECT_BR = cv::Point2f(250, 145);
const cv::Point2f  RECT_BL = cv::Point2f(0, 145);

int main() {
	/*----- Min and Max HSV values for thesholding.
	*----- Then creating track bars to adjust these values ------*/
	bool manualCalibrate = false;
	bool autoCalibrate = false;
	bool perspectiveCalibrate = false;
	cv::Point2f perspectiveTransActualPoints[4] = { cv::Point2f(0, 0),cv::Point2f(145, 0),cv::Point2f(145, 250),cv::Point2f(0, 250) }; //Default Values
	int currentPerspectivePoint = -1;
		
	int hueLow = 32;
	int hueHigh = 120;
	int satLow = 18;
	int satHigh = 149;
	int valLow = 98;
	int valHigh = 201;

	// declare windows
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	cv::resizeWindow("Original", WINDOW_W, WINDOW_H); //
	cv::namedWindow("Transformed", CV_WINDOW_AUTOSIZE);
	cv::resizeWindow("Transformed", WINDOW_W, WINDOW_H); //Transformed
	cv::namedWindow("Thresh", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	cv::namedWindow("HSV", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	cv::namedWindow("Contours", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	cv::VideoCapture capWebcam(WEB_CAM);		// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

	if (WEBCAM)
	{
		if (capWebcam.isOpened() == false) {			// check if VideoCapture object was associated to webcam successfully
			std::cout << "error: capWebcam not accessed successfully\n\n";	// if not, print error message to std out
			SAVE_IMAGES = false;
			return(0);														// and exit program
		}
	}

	cv::Mat frameOriginal;		// input image
	cv::Mat frameTransformed;		// input image
	cv::Mat im_with_keypoints;	
	cv::Mat frameBlurred;			// intermediate blured image
	cv::Mat frameThresh;			// Thresholded Image
	cv::Mat frameHsv;				// Hue, Sat, Value
	cv::Vec3b intensity;		// for RGB intensity values

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	int count = 0;
	char charCheckForKey = 0;

	while (charCheckForKey != 27 && (WEBCAM ? capWebcam.isOpened() : true) && (count < 100)) {
		// until the Esc key is pressed or webcam connection is lost

		if (WEBCAM) {
			bool blnFrameReadSuccessfully = capWebcam.read(frameOriginal);		// get next frame

			if (!blnFrameReadSuccessfully || frameOriginal.empty()) {		// if frame not read successfully
				std::cout << "error: frame not read from webcam\n";		// print error message to std out
				break;													// and jump out of while loop
			}
		}
		else {
			frameOriginal = cv::imread(FILE_NAME + std::to_string(count) + ".jpg", 1);
		}

		// convert to HSV color space
		cv::cvtColor(frameOriginal, frameHsv, CV_BGR2HSV);
		
		cv::GaussianBlur(frameHsv,			// input image
			frameBlurred,							// output image
			cv::Size(5, 5),						    // smoothing window width and height in pixels
			1.8);									// sigma value, determines how much the image will be blurred

		//----- Apply Threshold
		/*----- the erode and dilate to clear up background noise.-----*/
		cv::inRange(frameHsv, cv::Scalar(hueLow, satLow, valLow), cv::Scalar(hueHigh, satHigh, valHigh), frameThresh);
		cv::erode(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		//Show frames
		cv::imshow("Thresh", frameThresh);
		cv::imshow("HSV", frameHsv);
		cv::imshow("Contours", frameThresh);
		
		//Find counours
		/*
		cv::findContours(frameThresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		std::vector<std::vector<cv::Point>> hull(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			cv::convexHull(cv::Mat(contours[i]), hull[i], false);
		}

		std::vector<std::vector<cv::Point>> contour_polygons(contours.size());
		std::vector<cv::Rect> bounding_boxs(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			cv::approxPolyDP(cv::Mat(hull[i]), contour_polygons[i], 3, true);
			bounding_boxs[i] = cv::boundingRect(cv::Mat(contour_polygons[i]));
		}

		//draw conturs on origional image
		// Filtering bounding boxes by size	
		for (int i = 0; i < contours.size(); i++) {
			float area = bounding_boxs[i].height * bounding_boxs[i].width;
			if (area > 1000) {
				cv::rectangle(frameOriginal, bounding_boxs[i].tl(), bounding_boxs[i].br(), cv::Scalar(0, 0, 255), 8, 0);
			}
		}
		*/

		if (SAVE_IMAGES)
		{
			cv::imwrite("C:/Users/Josh/Desktop/Images/frame_" + std::to_string(count) + ".jpg", frameOriginal);
			count++;
		}

		int roi_x = 200;
		int roi_y = 100;
		int roi_w = 10;
		int roi_h = 10;
		cv::Vec3f vals = GetValsAtRoi(roi_x, roi_y, roi_w, roi_h, frameOriginal);
		//printf("B: %f, G: %f, R: %f\n", vals[0], vals[1], vals[2]);

		if (vals[2] < 50) {
			SendByte(255);
		}

		cv::imshow("Original", frameOriginal);
		cv::Point2f actual[] = {cv::Point2f(232, 77), cv::Point2f(443, 66), cv::Point2f(430, 145), cv::Point2f(245, 155)};
		cv::Point2f desired[] = { cv::Point2f(200, 100),cv::Point2f(450, 100),cv::Point2f(450, 245),cv::Point2f(200, 245) };
		PerformPerspectiveTransform(frameOriginal, frameTransformed, cv::Point(640, 480), actual, desired);
		

		if (manualCalibrate) {
			ManualCalibrate(&hueLow, &hueHigh, &satLow, &satHigh, &valLow, &valHigh);
		}
		if (autoCalibrate) {
			// Get Mouse Poseition
			cvSetMouseCallback("Transformed", MouseMoveCallback, NULL);
			int height = 10;
			int width = 10;
			int x1 = mousePose.x - (width / 2);
			int x2 = mousePose.x + (width / 2);
			int y1 = mousePose.y - (height / 2);
			int y2 = mousePose.y + (height / 2);
			cv::Rect roi = cv::Rect(x1, y1, width, height);
			AutoCalibrate(&hueLow, &hueHigh, &satLow, &satHigh, &valLow, &valHigh, x1, y1, width, height, frameTransformed);
			cv::rectangle(frameTransformed, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2, 0);
		}
		if (perspectiveCalibrate)
		{
			cvSetMouseCallback("Original", MouseLClickCallback, NULL);
		}

		cv::imshow("Transformed", frameTransformed);
		charCheckForKey = cv::waitKey(1);		// delay (in ms) and get key press, if any
		//printf("Key Code: %i\n", charCheckForKey);
		switch (charCheckForKey)
		{
		case 109: //m
			manualCalibrate = !manualCalibrate;
			printf("man:\n");
			break;
		case 99: //c
			autoCalibrate = !autoCalibrate;
			printf("auto\n");
			break;
		case 112: //c
			perspectiveCalibrate = !perspectiveCalibrate;
			printf("perspective\n");
			break;
		case 32:
			if (!WEBCAM) {
				count++;
				printf("Frame: %i\n", count);
				if (count > 100) { count = 0; }
			}
		default:
			break;
		}

	}	// end while

	return(0);
}

void SendByte(int toSend)
{
	array<unsigned char>^ texBufArray = gcnew array<unsigned char>(1);
	int baudRate = 9600;
	// robot interpreter box settings
	SerialPort^ robot_int;
	robot_int = gcnew SerialPort(COM_PORT, baudRate);
	// open port
	try
	{
		// Open port to robot interpreter box
		robot_int->Open();

		// Set number to send to the port
		texBufArray[0] = toSend;

		// Write number to the port
		robot_int->Write(texBufArray, 0, 1);

		// close port to robot interpreter box
		robot_int->Close();
	}
	catch (IO::IOException^ e)
	{
		Console::WriteLine(e->GetType()->Name + ": Port is not ready");
	}
}

float* ClassifyBeer(cv::Mat &frame) {
	float bays[3];
	int x = 238;
	int y = 208;
	int hei = 20;
	int wid = 20;
	cv::Vec3f bay = GetValsAtRoi(x, y, wid, hei, frame);
	bays[0] = bay[0] + bay[1] + bay[2] / 3;
	x = 338;
	y = 207;
	bay = GetValsAtRoi(x, y, wid, hei, frame);
	bays[1] = bay[0] + bay[1] + bay[2] / 3;
	x = 431;
	y = 200;
	bay = GetValsAtRoi(x, y, wid, hei, frame);
	bays[2] = bay[0] + bay[1] + bay[2] / 3;
	return bays;
}

void ManualCalibrate(int* hueLow, int* hueHigh, int* satLow, int* satHigh, int* valLow, int* valHigh) {
	cv::createTrackbar("Hue Low", "Original", hueLow, 179);
	cv::createTrackbar("Hue High", "Original", hueHigh, 179);
	cv::createTrackbar("Saturation Low", "Original", satLow, 255);
	cv::createTrackbar("Saturation High", "Original", satHigh, 255);
	cv::createTrackbar("Value Low", "Original", valLow, 255);
	cv::createTrackbar("Value High", "Original", valHigh, 255);
}

cv::Vec3f GetValsAtRoi(int &x1, int &y1, int &width, int &height, cv::Mat &frame) {
	cv::Vec3f result;
	if (y1 <= 0) { y1 = 0; }
	if (y1 + (height) >= WINDOW_H) { y1 = WINDOW_H - (height); }
	if (x1 <= 0) { x1 = 0; }
	if (x1 + (width) >= WINDOW_W) { x1 = WINDOW_W - (width); }
	for (int row = y1; row < height + y1; row++)
	{
		for (int col = x1; col < width + x1; col++)
		{
			cv::Vec3b pix = frame.at<cv::Vec3b>(row, col);
			result[0] += pix.val[0];
			result[1] += pix.val[1];
			result[2] += pix.val[2];
		}
	}
	result[0] = result[0] / (width * height);
	result[1] = result[1] / (width * height);
	result[2] = result[2] / (width * height);
	cv::rectangle(frame, cv::Point(x1, y1), cv::Point((x1 + width), (y1 + height)), cv::Scalar(0, 0, 255), 2, 0);
	return result;
}

void AutoCalibrate(int *hueLow, int *hueHigh, int *satLow, int *satHigh, int *valLow, int *valHigh, int &x1, int &y1, int &width, int &height, cv::Mat &frame) {
	cv::Vec3f vals = GetValsAtRoi(x1, y1, width, height, frame);
	printf("H: %f, S: %f, V: %f \n", vals[0], vals[1], vals[2]);
	int plusMinus = 70;
	*hueLow = vals[0] - plusMinus;
	*hueHigh = vals[0] + plusMinus;
	*satLow = vals[1] - plusMinus;
	*satHigh = vals[1] + plusMinus;
	*valLow = vals[2] - plusMinus;
	*valHigh = vals[2] + plusMinus;
}

void MouseMoveCallback(int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_MOUSEMOVE)
	{
		mousePose.x = x;
		mousePose.y = y;
	}
}

void MouseLClickCallback(int event, int x, int y, int flags, void *userdata)
{
	if (event == cv::EVENT_LBUTTONDBLCLK)
	{
		printf("X: %i, Y: %i\n", x, y);
	}
}
 
void PerformPerspectiveTransform(cv::Mat &frameIn, cv::Mat &frameOut, cv::Point &size, cv::Point2f actual[], cv::Point2f desired[] )
{
	cv::Mat transform(3, 3, CV_32FC1);
	transform = cv::getPerspectiveTransform(actual, desired);
	cv::warpPerspective(frameIn, frameOut, transform, size);
}

//cv::Point2f actual[4] = { cv::Point2f(43,18), cv::Point2f(280,40), cv::Point2f(19,233), cv::Point2f(304,200) };
//cv::Point2f desired[4] = { cv::Point2f(0,0), cv::Point2f(320,0), cv::Point2f(0,240),  cv::Point2f(320,240) };
//205x275