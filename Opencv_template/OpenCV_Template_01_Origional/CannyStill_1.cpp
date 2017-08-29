// CannyWebcam.cpp

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<iostream>

///////////////////////////////////////////////////////////////////////////////////////////////////
int main() {
	cv::VideoCapture capWebcam(0);		// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

	if (capWebcam.isOpened() == false) {			// check if VideoCapture object was associated to webcam successfully
		std::cout << "error: capWebcam not accessed successfully\n\n";	// if not, print error message to std out
		return(0);														// and exit program
	}

	cv::Mat frameOriginal;		// input image
	cv::Mat frameGrayscale;		// grayscale of input image
	cv::Mat frameBlurred;			// intermediate blured image
	cv::Mat frameCanny;			// Canny edge image
	cv::Mat frameHsv;				// Hue, Sat, Value
	cv::Vec3b intensity;		// for RGB intensity values

	char charCheckForEscKey = 0;

	while (charCheckForEscKey != 27 && capWebcam.isOpened()) {		// until the Esc key is pressed or webcam connection is lost
		bool blnFrameReadSuccessfully = capWebcam.read(frameOriginal);		// get next frame

		if (!blnFrameReadSuccessfully || frameOriginal.empty()) {		// if frame not read successfully
			std::cout << "error: frame not read from webcam\n";		// print error message to std out
			break;													// and jump out of while loop
		}

		cv::cvtColor(frameOriginal, frameGrayscale, CV_BGR2GRAY);		// convert to grayscale
		cv::cvtColor(frameOriginal, frameHsv, CV_BGR2HSV);		// convert to HSV color space

		cv::GaussianBlur(frameGrayscale,			// input image
			frameBlurred,							// output image
			cv::Size(5, 5),						// smoothing window width and height in pixels
			1.8);								// sigma value, determines how much the image will be blurred

		cv::Canny(frameBlurred,			// input image
			frameCanny,					// output image
			50,							// low threshold
			100);						// high threshold
		
		//Frame w:640, h:480
		for (int i = 100; i < frameOriginal.rows - 100; i++)
		{
			for (int j = 100; j < frameOriginal.cols - 100 ; j++)
			{
				frameOriginal.at<cv::Vec3b>(i, j)[0] = 0; // Blue= 0
				//frameOriginal.at<cv::Vec3b>(i, j)[1] = 0; // Green = 0
				frameOriginal.at<cv::Vec3b>(i, j)[2] = 0; // Red = 0
			}
		}
		
		// declare windows
		cv::namedWindow("imgOriginal", CV_WINDOW_NORMAL);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
		cv::namedWindow("imgCanny", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
		cv::namedWindow("imgHSV", CV_WINDOW_NORMAL);		// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
															
															// CV_WINDOW_AUTOSIZE is the default
		cv::imshow("imgOriginal", frameOriginal);		// show windows
		cv::imshow("imgCanny", frameCanny);
		cv::imshow("imgHSV", frameHsv);

		charCheckForEscKey = cv::waitKey(1);		// delay (in ms) and get key press, if any
	}	// end while

	return(0);
}
