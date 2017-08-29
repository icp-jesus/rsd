// CannyWebcam.cpp

#define COM_PORT "COM5"
#define WINDOW_W 640
#define WINDOW_H 480
#define BOX_W 300
#define BOX_H 200
#define WEB_CAM  1// (0) for integrated webcam (1) for USB webcam
#define CAM_OR_STILL true //webcam on (true) stills from file (false)
#define SAVE_TO_FILE false //
#define NOPE 8
#define BAY_1 1
#define BAY_2 2
#define BAY_3 4
#define WAIT  0

#include"stdafx.h"
#using <System.dll>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <string.h>
#include <Windows.h>
#include <fstream>

#include<io.h>
#using <mscorlib.dll>
using namespace System;
using namespace System::IO::Ports;

const int serialBayCommands[] = { NOPE, BAY_1, BAY_2, BAY_3, WAIT };

enum BeerType {
	no_beer,
	blue,
	green,
	red
};

enum Bay {
	not_in_bay = 0,
	one = 1,
	two = 2,
	three = 3
};

struct Beer {
	std::string name;
	BeerType type;
	int bay;
	int serialCommand;
	bool availiable;
	int meanHue;
	double stdDev;
};

//Initalise NO_BEER_CONST vector
const Beer BEER_NO_BEER = { "cant find beer",  no_beer,  not_in_bay, serialBayCommands[0], false, -1, -1};
const int STD_DEVS = 5;

//Regions of interest
const int ROI_W = 20;
const int ROI_H = 20;

//----- Forward Declarations -----
void SendByte(int toSend);
cv::Vec3i GetHueStatsAtRoi(int x1, int y1, int width, int height, cv::Mat &HSVframe, bool stdDevandRangeToo);
void DrawROIs(cv::Mat &frame, std::vector<cv::Point2f> ROIs, int w, int h);
void GetAllRGBVals(int &x1, int &y1, int &width, int &height, cv::Mat &frame, std::vector<cv::Vec3i> &result);
void ManualCalibrate(int* hueLow, int* hueHigh, int* satLow, int* satHigh, int* valLow, int* valHigh);
void AutoCalibrate(cv::Mat &HSVframe, std::vector<Beer> &beers, std::vector<cv::Point2f> rois);
void MouseMoveCallback(int event, int x, int y, int flags, void *userdata);
void MouseLClickCallback(int event, int x, int y, int flags, void *userdata);
int SelectBeers(BeerType beerType, std::vector<Beer> beerVec);
void BeerMeBeerBot(std::vector<Beer> beerOrder);
void PrintAvailiableBeers(std::vector<Beer> beerArray);
void DrawBayLables(cv::Mat &frame, std::vector<Beer> beerArray, std::vector<cv::Point2f> bayLocs);
std::vector<Beer> ClassifyBeer(cv::Mat &frame, std::vector<cv::Point2f> ROIArray, std::vector<Beer> availiableBeers);
void MonitorBays(std::vector<Beer> beersInBays, std::vector<BeerType> &previousBeerArrangement);
void LogData(std::vector<cv::Vec3i> colors);
cv::Point mousePose = cv::Point(50, 50); // I know this is naughty, but I can't get the bloody callback function to update the pointer!!!

//debugging
bool WEBCAM = CAM_OR_STILL;
bool SAVE_IMAGES = SAVE_TO_FILE;

int main() {
	SendByte(WAIT);
#pragma region SetBoolsForModes
	system("Color 5E");
	bool manualROICalibrate = false;
	bool classifyBays = false;
	bool autoCalibrate = false;
	bool beerMe = false;
	bool monitor = false;
#pragma endregion

Beer beer_blue = { "blue",            blue,     not_in_bay, serialBayCommands[0], false,  107, 3 };
Beer beer_green = { "green",           green,    not_in_bay, serialBayCommands[0], false, 47, 2 };
Beer beer_red = { "red",             red,      not_in_bay, serialBayCommands[0], false, 171, 12 };
int STD_DEVS = 3;
std::vector<Beer> AVAILIABLE_BEERS = { beer_red, beer_green, beer_blue };

#pragma region DeclareWindowsSetUpCamera
	// declare windows
	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	cv::resizeWindow("Original", WINDOW_W, WINDOW_H);
	cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	cv::resizeWindow("HSV", WINDOW_W, WINDOW_H);
	cv::VideoCapture capWebcam(WEB_CAM);		// declare a VideoCapture object and associate to webcam, 0 => use 1st webcam


	if (WEBCAM)
	{
		if (capWebcam.isOpened() == false) {			// check if VideoCapture object was associated to webcam successfully
			std::cout << "error: capWebcam not accessed successfully\n\n";	// if not, print error message to std out
			SAVE_IMAGES =  false;
			return(0);														// and exit program
		}
	}
#pragma endregion

	cv::Mat frameOriginal;
	cv::Mat frameHSV;
	const std::string FILE_NAME_READ = "C:/Users/Josh/Desktop/Images/frame_";
	const std::string FILE_NAME_WRITE = "C:/Users/Josh/Desktop/Images_RobotRoom/frame_";
	int imageCaptureCount = 0;
	char charCheckForKey = 0;
	cv::Point previousPoint = mousePose; //this is for calibrating the perspective transform
	int bayCalibrationIdx = 0;
	
	std::vector<Beer> beersInBays;
	beersInBays.push_back(BEER_NO_BEER);
	beersInBays.push_back(BEER_NO_BEER);
	beersInBays.push_back(BEER_NO_BEER);
	std::vector<BeerType> previousBeerArrangement;
	previousBeerArrangement.push_back(no_beer);
	previousBeerArrangement.push_back(no_beer);
	previousBeerArrangement.push_back(no_beer);
//	BeerType beerSeletion = no_beer;
	int bayCheckStrikes = 0;
	std::vector<Beer> beerOrder; //for selecting beers from menue
	
	//Initalise bayROI's
	std::vector<cv::Point2f>  bayROIs = { cv::Point2f(264,62),cv::Point2f(310,66),cv::Point2f(355,71) };


	//Initislise reference points for persperctive transform
	//int perspectiveCalibrationIdx = 0;
	//std::vector<cv::Point2f> perspectiveTransformActual = { cv::Point2f(100,100),cv::Point2f(200,100),cv::Point2f(200,200),cv::Point2f(100,200) };
	//std::vector<cv::Point2f> perspectiveTransformDesired = { cv::Point2f(0,0),cv::Point2f(WINDOW_W,0),cv::Point2f(WINDOW_W,WINDOW_H),cv::Point2f(0,WINDOW_H) };

	//for logging colors
	std::vector<cv::Vec3i> colorLog;
	const int MAX_SAMPLES = 500;
	int colorLogCount = 0;
	bool logColor = false;
	
	while (charCheckForKey != 27 && (WEBCAM ? capWebcam.isOpened() : true) && (imageCaptureCount < 100)) {
		
#pragma region UserInfo
		//notes: 
		system("cls");
		printf("Keys:\n");
		printf("(m)anual calibrate regions of interest.");
		if (manualROICalibrate) { printf("*\n"); }
		else { printf("\n"); }

		printf("(c)lasify regions of interest.");
		if (classifyBays) { printf("*\n"); }
		else { printf("\n"); }

		printf("(a)uto calibrate.");
		if (autoCalibrate) { printf("*\n"); }
		else { printf("\n"); }

		printf("(b)eer me beer bot.");
		if (beerMe) { printf("*\n"); }
		else { printf("\n"); }
#pragma endregion
		
		// until the Esc key is pressed or webcam connection is lost
		switch (charCheckForKey)
		{
		case 109: //(m)anually calibrate bays
			manualROICalibrate = !manualROICalibrate;
			autoCalibrate = false;
			beerMe = false;
			printf("ROI Calibrate:\n");
			break;
		case 99: //(c)lasify bays
			classifyBays = !classifyBays;
			manualROICalibrate = false;
			autoCalibrate = false;
			beerMe = false;
			printf("clasify bays\n");
			break;
		case 98: //(b)eer me beer bot
			beerMe = !beerMe;
			manualROICalibrate = false;
			autoCalibrate = false;
			classifyBays = false;
			printf("Coming right up!\n");
			SendByte(WAIT);
			break;
		case 97: //(a)uto calubrate
			beerMe = false;
			manualROICalibrate = false;
			classifyBays = false;
			autoCalibrate = !autoCalibrate;
			SendByte(WAIT);
			break;
		case 102: //write to file
			logColor = true;
			break;
		case 32: //(space) moves to next frame
			if (!WEBCAM) {
				imageCaptureCount++;
				printf("Frame: %i\n", imageCaptureCount);
				if (imageCaptureCount > 100) { imageCaptureCount = 0; }
			}
		default:
			break;
		}

		if (WEBCAM) {
			bool blnFrameReadSuccessfully = capWebcam.read(frameOriginal);		// get next frame

			if (!blnFrameReadSuccessfully || frameOriginal.empty()) {		// if frame not read successfully
				std::cout << "error: frame not read from webcam\n";		    // print error message to std out
				break;													    // and jump out of while loop
			}
		}
		else {
			frameOriginal = cv::imread(FILE_NAME_READ + std::to_string(imageCaptureCount) + ".jpg", 1);
		}

		if (SAVE_IMAGES)
		{
			cv::imwrite(FILE_NAME_WRITE + std::to_string(imageCaptureCount) + ".jpg", frameOriginal);
			imageCaptureCount++;
		}
		cv::cvtColor(frameOriginal, frameHSV, CV_BGR2HSV);
		/*
		if (logColor) {
			int wid = 20;
			int hei = 20;
			for (std::vector<cv::Point2f>::iterator it = bayROIs.begin(); it != bayROIs.end(); ++it) {
				int x = it->x;
				int y = it->y;
				int x_cen = x - (wid / 2);
				int y_cen = y - (hei / 2);
				Beer newBeer;

				GetAllRGBVals(x_cen, y_cen, wid, hei, frameOriginal, colorLog);
				colorLogCount++;
				std::printf("DATA: %d", colorLogCount);

			}
			if (colorLogCount >= MAX_SAMPLES) {
				printf("WINNING!!!");
				LogData(colorLog);
				colorLogCount = 0;
				logColor = false;
			}
		}*/

#pragma region CLASSIFY_BAYS
		beersInBays = ClassifyBeer(frameHSV, bayROIs, AVAILIABLE_BEERS);
		MonitorBays(beersInBays, previousBeerArrangement);
#pragma endregion

		if (manualROICalibrate) {
			system("Color 1A");
			printf("___Manual Calibration Mode___\nDouble Click the regions of interest.\n");
			for (int i = 0; i <3; i++) {
				printf("ROI %i of 3: (%i, %i)\n", i+1, (int)bayROIs[i].x, (int)bayROIs[i].y);
			}
			
			cvSetMouseCallback("Original", MouseLClickCallback, NULL);
			if (previousPoint != mousePose) {
				printf("PT: %i (X: %i, Y: %i)\n", bayCalibrationIdx, mousePose.x, mousePose.y);
				bayROIs[bayCalibrationIdx] = mousePose;
				previousPoint = mousePose;
				printf("Bays %i Calibrated\n", bayCalibrationIdx);
				bayCalibrationIdx++;
				if (bayCalibrationIdx > 2) {
					//manualROICalibrate = false;
					bayCalibrationIdx = 0;
					printf("Bay calibration complete!!! You effing legend!!!\n");
				}
				previousPoint = mousePose;
				classifyBays = true;
			}
			//FIX ME!!!!
			for (int i = 0; i <= bayCalibrationIdx; i++)
			{
				cv::rectangle(frameOriginal, cv::Point(bayROIs[i].x, bayROIs[i].y), cv::Point((bayROIs[i].x + 2), (bayROIs[i].y + 2)), cv::Scalar(0, 0, 255), 2, 0);
			}
		}
		//BEER ME
		else if (beerMe) {
			PrintAvailiableBeers(beersInBays);
			if (charCheckForKey > 0) {
				switch (charCheckForKey){//enterKey
				case 49: // '1'
					beerOrder.push_back(beersInBays[0]);
					break;
				case 50: //'2'
					beerOrder.push_back(beersInBays[1]);
					break;
				case 51: //'3'
					beerOrder.push_back(beersInBays[2]);
					break;
				case 100: //'(d)elets'
					if (beerOrder.size() > 0) { beerOrder.pop_back(); }
					break;
				case 32: //'spacebar' abort
					SendByte(WAIT);
					beerOrder.clear();
					beerMe = false;
					break;
				case 13: //'enter'
					BeerMeBeerBot(beerOrder);
					beerOrder.clear();
					beerMe = false;
					monitor = true;
					break;
				default:
					//nothing
					break;
				}
			}
			
			std::printf("Your order:     (d)elete last entry, (space) abort!\n");
			for (std::vector<Beer>::iterator it = beerOrder.begin(); it != beerOrder.end(); ++it) {
				std::printf("%s\n", it->name.c_str());
			}
		}
		else if (autoCalibrate) {
			AutoCalibrate(frameHSV, AVAILIABLE_BEERS, bayROIs);
			printf("Red: mean(%i), stdDev:(%f)\n", AVAILIABLE_BEERS[0].meanHue, AVAILIABLE_BEERS[0].stdDev);
			printf("Blue: mean(%i), stdDev:(%f)\n", AVAILIABLE_BEERS[1].meanHue, AVAILIABLE_BEERS[1].stdDev);
			printf("Green: mean(%i), stdDev:(%f)\n", AVAILIABLE_BEERS[2].meanHue, AVAILIABLE_BEERS[2].stdDev);
		}
		else {
			system("Color 5E");
		}

		DrawROIs(frameOriginal, bayROIs, ROI_W, ROI_H);
		DrawBayLables(frameOriginal, beersInBays, bayROIs);
		cv::imshow("Original", frameOriginal);
		cv::imshow("HSV", frameHSV);

		charCheckForKey = cv::waitKey(50);		// delay (in ms) and get key press, if any
		//printf("Key Code: %i\n", charCheckForKey);
	}	// end while

	cv::destroyAllWindows();
	return(0);
}

void SendByte(int toSend){
	std::printf("Byte Sent: %i\n", toSend);
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

void PrintAvailiableBeers(std::vector<Beer> beerArray) {
	std::printf("Availiable Beers:\n");
	for (std::vector<Beer>::iterator it = beerArray.begin(); it != beerArray.end(); ++it) {
		if (it->availiable) {
			std::printf("(%d): %s\n", it->bay, it->name.c_str());
		}
		else {
			std::printf("(%d): %s\n", it->bay, "--");
		}
	}
}

void DrawBayLables(cv::Mat &frame, std::vector<Beer> beerArray, std::vector<cv::Point2f> bayLocs)
{
	int i = 0;
	for (std::vector<Beer>::iterator beer = beerArray.begin(); beer != beerArray.end(); ++beer)
	{
		cv::putText(frame, beer->name, cv::Point(bayLocs[i].x, bayLocs[i].y - (ROI_H/2) + 2), 1, 1, cv::Scalar(0, 0, 255), 1, 8, false);
		i++;
	}
}

std::vector<Beer> ClassifyBeer(cv::Mat &frame, std::vector<cv::Point2f> ROIArray, std::vector<Beer> availiableBeers) {
	std::vector<Beer> result;
	int bayNum = 1;
	for (std::vector<cv::Point2f>::iterator it = ROIArray.begin(); it != ROIArray.end(); ++it) {
		int x = it->x;
		int y = it->y;
		
		Beer newBeer = BEER_NO_BEER;
		cv::Vec3i bayHSV = GetHueStatsAtRoi(x, y, ROI_W, ROI_H, frame, false);
		printf("bay%i: H: %i, S: %i, V: %i\n", bayNum, bayHSV[0], bayHSV[1], bayHSV[2]);
		for (std::vector<Beer>::iterator b = availiableBeers.begin(); b != availiableBeers.end(); ++b)
		{
			if (abs(b->meanHue - bayHSV[0]) < (b->stdDev * STD_DEVS)) {
				newBeer.availiable = true;
				newBeer.bay = bayNum;
				newBeer.meanHue = b->meanHue;
				newBeer.name = b->name;
				newBeer.serialCommand = b->serialCommand;
				newBeer.stdDev = b->stdDev;
				newBeer.type = b->type;
			}
		}
		result.push_back(newBeer);
		bayNum++;
	}
	return result;
}

void MonitorBays(std::vector<Beer> beersInBays, std::vector<BeerType> &previousBeerArrangement) {
	int checkIdx = 0;
	for (std::vector<Beer>::iterator it = beersInBays.begin(); it != beersInBays.end(); ++it) {
		if (it->type != previousBeerArrangement[checkIdx])
		{
			SendByte(NOPE);
		}
		previousBeerArrangement[checkIdx] = it->type;
		checkIdx++;
	}
}

/*
void GetAllRGBVals(int &x1, int &y1, int &width, int &height, cv::Mat &frame, std::vector<cv::Vec3i> &result) {
	cv::Vec3i temp;
	if (y1 <= 0) { y1 = 0; }
	if (y1 + (height) >= WINDOW_H) { y1 = WINDOW_H - (height); }
	if (x1 <= 0) { x1 = 0; }
	if (x1 + (width) >= WINDOW_W) { x1 = WINDOW_W - (width); }
	for (int row = y1; row < height + y1; row++)
	{
		for (int col = x1; col < width + x1; col++)
		{
			cv::Vec3i  pix = frame.at<cv::Vec3b>(row, col);
			temp.val[0] = pix.val[0];
			temp.val[1] = pix.val[1];
			temp.val[2] = pix.val[2];
			//printf("B: %d, G: %d, R: %d", temp.val[0], temp.val[1], temp.val[2]);
			result.push_back(temp);
			//if (!((temp.val[0] == 0) || (temp.val[0] == 255) || (temp.val[1] == 0) || (temp.val[1] == 255) || (temp.val[2] == 0) || (temp.val[2] == 255))){

			//}
		}
	}

}
*/
void DrawROIs(cv::Mat &frame, std::vector<cv::Point2f> ROIs, int w, int h) {
	for (std::vector<cv::Point2f>::iterator i = ROIs.begin(); i != ROIs.end(); ++i) {
		int x = (int)i->x;
		int y = (int)i->y;
		if (y <= 0) { y = 0; }
		if (y + (h) >= WINDOW_H) { y = WINDOW_H - (h); }
		if (x <= 0) { x = 0; }
		if (x + (w) >= WINDOW_W) { x = WINDOW_W - (w); }
		cv::rectangle(frame, cv::Point(x, y), cv::Point((x + w), (y + h)), cv::Scalar(0, 0, 255), 2, 0);
	}
}

cv::Vec3i GetHueStatsAtRoi(int x1, int y1, int width, int height, cv::Mat &HSVframe, bool stdDevandRangeToo) {
	std::vector<int> hues;
	hues.reserve(width * height);

	cv::Vec3i result = { 0,0,0 };
	if (y1 <= 0) { y1 = 0; }
	if (y1 + (height) >= WINDOW_H) { y1 = WINDOW_H - (height); }
	if (x1 <= 0) { x1 = 0; }
	if (x1 + (width) >= WINDOW_W) { x1 = WINDOW_W - (width); }
	//printf("Rows: %i Cols: %i\n", frame.rows, frame.cols);
	//printf("Roi.X: %i, Row.Y: %i\n", x1, y1);

	for (int row = y1; row < height + y1; row++)
	{
		for (int col = x1; col < width + x1; col++)
		{
			cv::Vec3i  pix = HSVframe.at<cv::Vec3b>(row, col);
			result[0] += pix.val[0];
			result[1] += pix.val[1];
			result[2] += pix.val[2];
			if (stdDevandRangeToo) { hues.push_back(pix.val[0]); }
		}
	}
	result[0] = result[0] / (width * height); //meanHue
	result[1] = result[1] / (width * height);
	result[2] = result[2] / (width * height);
	float variance = 0.0f;
	int min = 255;
	int max = 0;
	if (stdDevandRangeToo) {
		for (std::vector<int>::iterator it = hues.begin(); it != hues.end(); ++it ){
			if (*it < min) {min = *it;}
			if (*it > max) { max = *it;}
			variance += (*it - result[0])*(*it - result[0]);
		}
		variance = variance / hues.size();
		result[1] = sqrt(variance);
		result[2] = max - min;//range
	}
	//printf("1: %i, 2: %i, 3: %i\n", result[0], result[1], result[2]);
	return result;
}

void AutoCalibrate(cv::Mat &HSVframe, std::vector<Beer> &beers, std::vector<cv::Point2f> rois) {
	cv::Vec3i vals;
	int idx = 0;
	printf("Place Red, Green, Blue in bays 1,2,3 respectivally\n");
	for (std::vector<cv::Point2f>::iterator r = rois.begin(); r != rois.end(); ++r) {
		vals = GetHueStatsAtRoi(r->x, r->y, ROI_W, ROI_H, HSVframe, true);
		beers[idx].meanHue = vals[0];
		beers[idx].stdDev = vals[1];
		idx++;
	}
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
		mousePose.x = x;
		mousePose.y = y;
		//printf("X: %i, Y: %i\n", x, y);
	}
}

void BeerMeBeerBot(std::vector<Beer> beerOrder) {
	int command = 0;
	for (std::vector<Beer>::iterator it = beerOrder.begin(); it != beerOrder.end(); ++it) {
		command += it->serialCommand;
	}
	SendByte(command);
	Sleep(100);
	SendByte(WAIT);
}
/*
void PerformPerspectiveTransform(cv::Mat &frameIn, cv::Mat &frameOut, cv::Point &size, std::vector<cv::Point2f> camPerspective, std::vector<cv::Point2f> desiredPerspective)
{
	cv::Mat transform(3, 3, CV_32FC1);
	transform = cv::getPerspectiveTransform(camPerspective, desiredPerspective);
	cv::warpPerspective(frameIn, frameOut, transform, size);
}
*/
void LogData(std::vector<cv::Vec3i> colors) {
	std::ofstream myfile;
	myfile.open("Red_Home_HSV.txt");
	for (std::vector<cv::Vec3i>::iterator it = colors.begin(); it != colors.end(); ++it) {
		std::string towrite = std::to_string(it->val[0]) + "\t" + std::to_string(it->val[1]) + "\t" + std::to_string(it->val[2]) + "\n";
		myfile << towrite;
	}
	myfile.close();
}


