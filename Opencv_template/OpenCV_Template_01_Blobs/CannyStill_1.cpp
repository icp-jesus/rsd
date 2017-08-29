// CannyWebcam.cpp

#include "stdafx.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<vector>
#include<string>
#include<iostream>
#using <System.dll>
#using<mscorlib.dll>

using namespace System::IO::Ports;
using namespace System;

//To Do, add calibration funcitonality
///////////////////////////////////////////////////////////////////////////////////////////////////
int main() {
	char charCheckForEscKey = 0;

	cv::VideoCapture capWebcam(1);		// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

	if (capWebcam.isOpened() == false) {			// check if VideoCapture object was associated to webcam successfully
		std::cout << "error: capWebcam not accessed successfully\n\n";	// if not, print error message to std out
		return(0);														// and exit program
	}

	cv::Mat frameOriginal;		// input image
	cv::Mat frameGrayscale;		// grayscale of input image
	cv::Mat frameBlurred;			// intermediate blured image
	//cv::Mat frameCanny;			// Canny edge image
	cv::Mat frameHsv;				// Hue, Sat, Value
	cv::Mat frameThresh;
	cv::Mat im_with_keypoints;
	
	cv::Vec3b intensity;		// for RGB intensity values

	/*----- Declare Windows -----*/
	cv::namedWindow("Original", CV_WINDOW_NORMAL);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	cv::namedWindow("HSV", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	cv::namedWindow("Thresh", CV_WINDOW_NORMAL);
	//cv::namedWindow("Keypoints", CV_WINDOW_NORMAL);

	/*----- Min and Max HSV values for thesholding. 
	 *----- Then creating track bars to adjust these values ------*/
	int hueLow = 40;
	int hueHigh = 84;
	int satLow = 40;
	int satHigh = 224;
	int valLow = 52;
	int valHigh = 255;

	cv::createTrackbar("Hue Low", "Original", &hueLow, 179);
	cv::createTrackbar("Hue High", "Original", &hueHigh, 179);
	cv::createTrackbar("Saturation Low", "Original", &satLow, 255);
	cv::createTrackbar("Saturation High", "Original", &satHigh, 255);
	cv::createTrackbar("Value Low", "Original", &valLow, 255);
	cv::createTrackbar("Value High", "Original", &valHigh, 255);
	
	//Setup blob detector
	cv::SimpleBlobDetector::Params params;
	//params.filterByColor = 1;				//Filter light parts of image
	//params.blobColor = 0;
	params.minThreshold = 0;				// Change thresholds
	params.maxThreshold = 10;
	params.filterByArea = true;				// Change area thresholds
	params.minArea = 3;
	params.maxArea = 10000;
	params.filterByCircularity = true;		// Filter by Circularity (
	params.minCircularity = 0.1;
	params.filterByConvexity = false;		// Filter by Convexity
	//params.minConvexity = 0.0;
	//params.maxConvexity = 1.0;
	params.filterByInertia = true;			// Filter by Inertia
	params.minInertiaRatio = 0.01;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
	std::vector<cv::KeyPoint> keypoints;
	
	
	/*----- Loop until esc key or webcam closes -----*/
	while (charCheckForEscKey != 27 && capWebcam.isOpened()) {
		bool blnFrameReadSuccessfully = capWebcam.read(frameOriginal);		// get next frame
				//iterator for frame capture
		for (int i = 1; i < 101; i++) {
			if (!blnFrameReadSuccessfully || frameOriginal.empty()) {		    // if frame not read successfully
				std::cout << "error: frame not read from webcam\n";		        // print error message to std out
				break;													        // and jump out of while loop
			}

			/*----- Convert origional to HSV -----*/
			cv::cvtColor(frameOriginal, frameHsv, CV_BGR2HSV);		            // convert to HSV color space
			// --- FRAME SAVING --- 
			/*
			cv::imwrite("c:/Users/josh/documents/output/date1/frameHSV" + std::to_string(i) + ".jpg", frameHsv);
			cv::imwrite("c:/Users/josh/documents/output/date2/frameOri" + std::to_string(i) + ".jpg", frameOriginal);
			*/


			/*-----  -----*/
			cv::GaussianBlur(frameHsv,			                       // input image
				frameBlurred,							// output image
				cv::Size(5, 5),						// smoothing window width and height in pixels
				1.8);								// sigma value, determines how much the image will be blurred


			/*----- Apply Threshold
			 *----- the erode and dilate to clear up background noise.-----*/
			cv::inRange(frameHsv, cv::Scalar(hueLow, satLow, valLow), cv::Scalar(hueHigh, satHigh, valHigh), frameThresh);
			cv::erode(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::dilate(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::dilate(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
			cv::erode(frameThresh, frameThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));



			/*----CANNY-----*/
			/*
			cv::Canny(frameBlurred,			// input image
				frameCanny,					// output image
				50,							// low threshold
				100);						// high threshold
			*/

			/*-----Color Shift------*/
			//Frame w:640, h:480
			/*
			for (int i = 100; i < frameOriginal.rows - 100; i++)
			{
				for (int j = 100; j < frameOriginal.cols - 100 ; j++)
				{
					frameOriginal.at<cv::Vec3b>(i, j)[0] = 0; // Blue= 0
					//frameOriginal.at<cv::Vec3b>(i, j)[1] = 0; // Green = 0
					frameOriginal.at<cv::Vec3b>(i, j)[2] = 0; // Red = 0
				}
			}
			*/

			//cv::imshow("Original", frameOriginal);		// show windows
			cv::imshow("HSV", frameHsv);
			cv::imshow("Thresh", frameThresh);

			// Detect blobs.
			frameThresh = cv::Scalar::all(255) - frameThresh;  //invert image
			detector->detect(frameThresh, keypoints);

			// Draw detected blobs as red circles.
			// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob

			cv::drawKeypoints(frameOriginal, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// Show blobs
			//for(std::vector<type>::itterator it = vect.being(); it!=vect.end(); ++it)
			for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
				std::string xAndY = "(" + std::to_string((int)it->pt.x) + ", " + std::to_string((int)it->pt.y) + ") " +
					"Area: " + std::to_string((int)it->size) + "px";
				cv::putText(im_with_keypoints, xAndY, it->pt, cv::HersheyFonts::FONT_HERSHEY_COMPLEX,
					0.5, cv::Scalar(0, 0, 255), 1, 8, false);
			}

			cv::imshow("Original", im_with_keypoints);

			charCheckForEscKey = cv::waitKey(1);		// delay (in ms) and get key press, if any

	}
		break;
	}	// end while

	return(0);
}
