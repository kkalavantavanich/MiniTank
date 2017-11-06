#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <string>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <KinectConnector.h>
#include <cstdint> 
#include "RobotConnector.h"
#include "easywsclient.hpp"
#include <algorithm>

#include <vector>

#include <regex>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#define MATH_PI 3.14159265358979323846264


#define Create_Comport "COM3"

using easywsclient::WebSocket;
static WebSocket::pointer wsk = NULL;

double vx = 0;
double vz = 0;
char direction = ' ';

typedef struct {
	int id;
	double x, y, z;
	double angle;
} gps_t;

typedef float belief_t;

std::smatch sm;
std::regex re("id: (\\d+)\tpos: (-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\tangle: (-?\\d+\\.\\d+)<br\\/>");
gps_t our_gps;
int semaphore_rst = 4;
int semaphore = semaphore_rst;

vector<double> ang_in_buff;
vector<double> x_in_buff;
vector<double> y_in_buff;
double ang_avg = 0;
double x_avg = 0;
double y_avg = 0;

bool enableMeasuring;

void handle_message(const std::string & message)
{
	if (semaphore == 0) {
		return;
	}
	semaphore--;
	if (message.find("end") != std::string::npos) {
		wsk->close();
		return;
	}
	std::string s = message;
	vector<gps_t> wsDataframe;
	while (std::regex_search(s, sm, re)) {
		int i = 0;
		gps_t g;
		for (auto x : sm) {
			switch (i) {
			case 1: // id
				g.id = std::stoi(x);
				break;
			case 2: // pos x
				g.x = std::stof(x);
				break;
			case 3: // pos y
				g.y = std::stof(x);
				break;
			case 4: // pos z
				g.z = std::stof(x);
				break;
			case 5: // angle
				g.angle = std::stof(x);
				break;
			}
			i++;
		}
		wsDataframe.push_back(g);
		if (g.id == 5) {
			our_gps = g;
			ang_avg += g.angle;
			x_avg += g.x;
			y_avg += g.y;
			ang_in_buff.push_back(g.angle);
			x_in_buff.push_back(g.x);
			y_in_buff.push_back(g.y);
		}
		s = sm.suffix().str();
	}
}

// updates in-place! (side-effect)
// receive only perceptual data (sensors)
// TODO: implement action data (motion)
void bayesFilter(cv::Mat belief, vector<cv::Point> gridCells, vector<double> probs) {
	// Create Buffer Matrix which is a float version of belief matrix
	// Values in buffer matrix are in range [0, 1].
	cv::Mat buff;
	buff.create(belief.rows, belief.cols, CV_32FC1);
	assert(buff.rows == belief.rows);
	assert(buff.cols == belief.cols);

	belief_t* beliefPtr = belief.ptr<belief_t>(0);
	float* buffPtr = buff.ptr<float>(0);
	for (int i = 0; i < buff.rows; i++) {
		for (int col = 0; col < buff.cols; ++col) {
			*buffPtr = (float)(((float)*beliefPtr) / 255.0);
			buffPtr++;
			beliefPtr++;
		}
	}



	double n = 0;
	int len = gridCells.size();

	// Check length of gridCells vs. probabilities
	if (len != probs.size()) {
		printf("Error in bayesFilter(): size of gridCells != size of probs.\n");
	}

	if (len == 0) {
		printf("Error in bayesFilter(): InvalidArgument len = 0\n");
	}

	// This is not exact implementation of Bayes Filter Algorithm
	// Design Decision is to keep unseen cells the same (not blured)
	float maxCell = 0;
	for (int i = 0; i < len; i++) {
		cv::Point p = gridCells[i];
		double prob = probs[i];
		float* cellPtr = buff.ptr<float>(p.y, p.x);
		uchar* beliefCellPtr = belief.ptr<uchar>(p.y, p.x);
		if (false) {
			printf("pos=(%d, %d), *belCellPtr = %d, *cellPtr = %f, prob = %lf, ", p.x, p.y, *beliefCellPtr, *cellPtr, prob);
		}
		(*cellPtr) *= prob;
		if (false) {
			printf("afterCellPtr = %f\n", *cellPtr);
		}
		n += (*cellPtr);
		maxCell = maxCell > *cellPtr ? maxCell : *cellPtr;
	}


	//printf("n = %lf\n", n);
	if (n == 0) {
		//printf("error in bayes filter: n == 0 causing DivideByZeroError\n");
		// do nothing 
		return;
	}
	else {
		n = 1.0 / n;
	}

	for (int i = 0; i < len; i++) {
		cv::Point p = gridCells[i];
		int val = (*buff.ptr<float>(p.y, p.x)) * 255 / maxCell;
		if (val > 255 || val < 0) {
			printf("error in bayes filter: val overflow at pos(%d, %d) val=%d.\n", p.x, p.y, val);
		}
		(*belief.ptr<belief_t>(p.y, p.x)) = val;
	}
}


void bayesFilter2(cv::Mat belief, cv::Mat nextProbs) {
	// Create Buffer Matrix which is a float version of belief matrix
	// Values in buffer matrix are in range [0, 1].
	cv::Mat buff;
	buff.create(belief.rows, belief.cols, CV_32FC1);
	assert(buff.rows == belief.rows);
	assert(buff.cols == belief.cols);

	double n = 0;

	belief_t* beliefPtr = belief.ptr<belief_t>(0);
	float* buffPtr = buff.ptr<float>(0);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; ++col) {
			*buffPtr = (float)(((float)*beliefPtr) / 255.0);
			buffPtr++;
			beliefPtr++;
		}
	}

	if (nextProbs.rows != buff.rows || nextProbs.cols != buff.cols) {
		printf("Error: buff size != nextProbs size");
	}

	float max = 0;
	buffPtr = buff.ptr<float>(0);
	float* nextProbPtr = nextProbs.ptr<float>(0);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			if (false) {
				printf("pos=(%d, %d), buff = %f, nextProb = %f\n", row, col, *buffPtr, *nextProbPtr);
			}
			(*buffPtr) *= (*nextProbPtr);
			n += (*buffPtr);
			buffPtr++;
			nextProbPtr++;
			max = max >(*buffPtr) ? max : (*buffPtr);
		}
	}

	//printf("n = %lf\n", n);
	if (n == 0) {
		//printf("error in bayes filter: n == 0 causing DivideByZeroError\n");
		// do nothing 
		return;
	}
	else {
		n = 1.0 / n;
	}

	buffPtr = buff.ptr<float>(0);
	beliefPtr = belief.ptr<belief_t>(0);

	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			int val = (*buffPtr) * 255.0 / max;
			if (val > 255 || val < 0) {
				printf("error in bayes filter: val overflow at val=%d.\n", val);
			}
			*beliefPtr = val;
			buffPtr++;
			beliefPtr++;
		}
	}

}

void bayesFilter3(cv::Mat belief, cv::Mat nextProbs) {
	// Create Buffer Matrix which is a float version of belief matrix
	// Values in buffer matrix are in range [0, 1].
	cv::Mat buff;
	buff.create(belief.rows, belief.cols, CV_32FC1);
	assert(buff.rows == belief.rows);
	assert(buff.cols == belief.cols);

	belief_t* beliefPtr = belief.ptr<belief_t>(0);
	float* buffPtr = buff.ptr<float>(0);
	printf("bayesFilter3: buff.rows = %d, buff.cols = %d\n", buff.rows, buff.cols);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; ++col) {
			*buffPtr = (float)(((float)*beliefPtr) / 255.0);
			buffPtr++;
			beliefPtr++;
		}
	}

	if (nextProbs.rows != buff.rows || nextProbs.cols != buff.cols) {
		printf("Error: buff size != nextProbs size");
	}

	buffPtr = buff.ptr<float>(0);
	float* nextProbPtr = nextProbs.ptr<float>(0);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			if (false) {
				printf("pos=(%d, %d), buff = %f, nextProb = %f, ", row, col, *buffPtr, *nextProbPtr);
			}
			(*buffPtr) = (*nextProbPtr)*(*buffPtr) /
				(((1 - (*nextProbPtr))*(1 - (*buffPtr))) + ((*nextProbPtr)*(*buffPtr)));
			(*buffPtr) = (*buffPtr) < 0.08 ? 0.08 : (*buffPtr);
			if (false) {
				printf("nextBuff = %f\n", *buffPtr);
			}
			buffPtr++;
			nextProbPtr++;
		}
	}

	buffPtr = buff.ptr<float>(0);
	beliefPtr = belief.ptr<belief_t>(0);

	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			int val = (*buffPtr) * 255.0;
			if (val > 255 || val < 0) {
				printf("error in bayes filter: val overflow at val=%d.\n", val);
			}
			*beliefPtr = val;
			buffPtr++;
			beliefPtr++;
		}
	}

}

bool closeSignal = false;

void close() {
	closeSignal = true;
	wsk->close();
	while (wsk->getReadyState() != wsk->CLOSED);
}

BOOL CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the CTRL-C signal. 
	case CTRL_C_EVENT:
		printf("Ctrl-C event\n\n");
		close();
		Beep(750, 300);
		return(TRUE);

		// CTRL-CLOSE: confirm that the user wants to exit. 
	case CTRL_CLOSE_EVENT:
		Beep(600, 200);
		printf("Ctrl-Close event\n\n");
		return(TRUE);

		// Pass other signals to the next handler. 
	case CTRL_BREAK_EVENT:
		Beep(900, 200);
		printf("Ctrl-Break event\n\n");
		return FALSE;

	case CTRL_LOGOFF_EVENT:
		Beep(1000, 200);
		printf("Ctrl-Logoff event\n\n");
		return FALSE;

	case CTRL_SHUTDOWN_EVENT:
		Beep(750, 500);
		printf("Ctrl-Shutdown event\n\n");
		return FALSE;

	default:
		return FALSE;
	}
}

int main() {
	std::cout << "starting...\n";
	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE))
	{
		printf("\nERROR: Could not set control handler");
		return 1;
	}


	// Robot Initialization
	/*CreateData	robotData;
	RobotConnector	robot;
	if (!robot.Connect(Create_Comport))
	{
	cout << "Error : Can't connect to robot @" << Create_Comport << endl;
	return -1;
	}
	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");
	*/

	// Kinect Initialization
	KinectConnector kin = KinectConnector();
	if (!kin.Connect()) return 1;
	std::cout << "initialized kinect\n";

	// Grid Variables
	double MILLIS_PER_GRID = 40;				// Cannot do less than 20 due to grid rough datatype size limit (uint16_t)
	double ROUGH_MILLIS_PER_GRID = 160;
	int GRID_HEIGHT = 7200 / MILLIS_PER_GRID;	// world size in mm
	int GRID_WIDTH = 9600 / MILLIS_PER_GRID;	// world size in mm
	int ROUGH_GRID_HEIGHT = 7200 / ROUGH_MILLIS_PER_GRID;	// world size in mm
	int ROUGH_GRID_WIDTH = 9600 / ROUGH_MILLIS_PER_GRID;	// world size in mm
	std::cout << "GRID SIZE = " << GRID_HEIGHT << "x" << GRID_WIDTH << "\n";
	int GRID_VIEW_HEIGHT = 480;
	int GRID_VIEW_WIDTH = 640;
	cv::namedWindow("GRID VIEW", cv::WINDOW_NORMAL);
	cv::resizeWindow("GRID VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);
	cv::Mat grid;											 // Main Grid Object (Greyscale)
	cv::Mat gridView;										 // Grid Object use for viewing (RGB)
	cv::Mat gridInt;
	cv::Mat gridRough;										// Use for motion planning
	cv::Mat cBuff;
	cv::Mat cSpace;											// (0 or 1) indicating C-Space

	grid = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_32FC1, cv::Scalar(127));
	gridInt = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_8UC1, cv::Scalar(127));
	gridView = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_8UC3);
	gridRough = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_16UC1, cv::Scalar(0));
	cBuff = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC1, cv::Scalar(0));
	cSpace = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC1, cv::Scalar(0));

	// Get from GPS
	cv::Point robotPoint(GRID_WIDTH / 2, GRID_HEIGHT / 2);	 // Current robot Position in Grid
	long double robotDirection = 0.0;						 // Robot Direction in range (-180, 180)
	int robotSize = 330;

	double KINECT_FOV_H = NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
	double KINECT_FOV_V = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV;

	// Debugging Variables
	int loop_index = 0;				// Count how many loops have passed, use for debugging.
									//int dt = robotData.distance;

	// Enable Measuring
	enableMeasuring = false;

									// Web Socket Initialization	
	std::cout << "initializing ws...\n";
#ifdef _WIN32
	INT rc;
	WSADATA wsaData;

	rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (rc) {
		printf("WSAStartup Failed.\n");
		return 1;
	}
#endif

	wsk = WebSocket::from_url("ws://192.168.1.59:8081/pose");
	assert(wsk);

	//while (wsk->getReadyState() != WebSocket::CLOSED) {

	//	wsk->send(" ");
	//	wsk->poll(-1);
	//	wsk->dispatch(handle_message);
	//std::cout << "OUR_GPS: " << our_gps.id << " pos=(" << our_gps.x << ", " << our_gps.y << "), ang=" << our_gps.angle << "\n";
	//	Sleep(100);
	//}
	//delete wsk;
	//#ifdef _WIN32
	//WSACleanup();
	//#endif


	cv::Mat nextProbs = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_32FC1, cv::Scalar(0.5));

	// Main Loop // 
	while (!closeSignal) {
		// *_in_buff is used as median of 4 values
		x_avg = 0;
		y_avg = 0;
		ang_avg = 0;
		x_in_buff.clear();
		y_in_buff.clear();
		ang_in_buff.clear();
		semaphore = semaphore_rst;

		for (int i = 0; i < 4; i++) {
			
		}
		int sem_cnt = 0;
		while (semaphore != 0) {
			if (sem_cnt < semaphore_rst) {
				sem_cnt++;
				wsk->send("GET");
				Sleep(10);
			}
			wsk->poll(-1);
			wsk->dispatch(handle_message);
			Sleep(10);
		}


		std::sort(x_in_buff.begin(), x_in_buff.end());
		std::sort(y_in_buff.begin(), y_in_buff.end());
		std::sort(ang_in_buff.begin(), ang_in_buff.end());

		for (auto it = x_in_buff.begin(); it != x_in_buff.end(); it++) {
			std::cout << *it << " ";
		}
		std::cout << "\n";
		x_avg = (x_in_buff[1] + x_in_buff[2] ) / 2;
		y_avg = (y_in_buff[1] + y_in_buff[2]) / 2;
		ang_avg = (ang_in_buff[1] + ang_in_buff[2]) / 2;

		printf("(_main.cpp:457) <%d> Updated ANG_AVG from GPS\n", loop_index);

		nextProbs.setTo(cv::Scalar(0.5));

		cv::Mat depthImg;
		cv::Mat colorImg;
		cv::Mat indexImg;
		cv::Mat pointImg;

		// Get Data from Kinect
		kin.GrabData(depthImg, colorImg, indexImg, pointImg);
		printf("(_main.cpp:457) <%d> Grabbed Data from Kinect.\n", loop_index);
		//if (!robot.ReadData(robotData))
		//cout << "ReadData Fail" << endl;

		// Make Center of ColorImg Red
		for (int theta = -20; theta <= 20; theta++) {
			if (theta == 0) {
				//double fd = (double)theta * colorImg.cols / NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
				long double fd = 2 * NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * tan((long double) theta * MATH_PI / 180.0);
				uchar* colorImgCenter = colorImg.ptr<uchar>(479, 639 - fd);
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				colorImgCenter = colorImg.ptr<uchar>(480, 639 - fd);
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
			}
			else {
				double fd = (double)theta * colorImg.cols / 60;
				uchar* colorImgCenter = colorImg.ptr<uchar>(479, 639 - fd);
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				colorImgCenter = colorImg.ptr<uchar>(480, 639 - fd);
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
			}
		}

		printf("(_main.cpp:500) <%d> Set color dots for colorImg\n", loop_index);
		
		imshow("depthImg", depthImg);
		imshow("colorImg", colorImg);

		printf("(_main.cpp:505) <%d> Showed colorImg and depthImg\n", loop_index);

		robotPoint.x = (x_avg * 10 / MILLIS_PER_GRID) + GRID_WIDTH / 2;
		robotPoint.y = (y_avg * 10 / MILLIS_PER_GRID) + GRID_HEIGHT / 2;
		printf("(_main.cpp:523) <%d> ang = %lf, x = %lf, y = %lf\n", loop_index, ang_avg, x_avg, y_avg);

		//imshow("indexImg", indexImg);
		//imshow("pointImg", pointImg);
		vector<cv::Point> updatePoints;
		vector<double> updateProbs;
		vector<cv::Point> objectPoints;
		for (int theta = -20; theta <= 20; theta++) {
			// theta valid from approx. -30 to 30
			// pixel_interested = center_pixel + (focal_length * 2 * tan(alpha))
			//double fd = (double)theta * depthImg.rows / NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV;
			long double fd = 2 * NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * tan((long double) theta * MATH_PI / 180.0);
			int16_t averagedPoints[4] = { *depthImg.ptr<int16_t>(239, 319 - fd),
				*depthImg.ptr<int16_t>(239, 320 - fd),
				*depthImg.ptr<int16_t>(240, 319 - fd),
				*depthImg.ptr<int16_t>(240, 320 - fd) };
			int averagedDepth = 0;
			int validPointCount = 0;
			//std::cout << "centerPoints: ";
			for (int i = 0; i < 4; i++) {
				//std::cout << centerPoints[i] << " ";
				// Kinect works from 0.5 - 4.5 meters
				if (averagedPoints[i] > 500 && averagedPoints[i] < 3000) {
					averagedDepth += averagedPoints[i];
					validPointCount++;
				}
			}
			if (validPointCount != 0) {
				averagedDepth /= validPointCount;
			}
			else {
				averagedDepth = 0;
			}

			// map gps to robot world grid
			robotDirection = ang_avg + theta;
			if (robotDirection > 180) {
				robotDirection -= 360;
			}
			else if (robotDirection < -180) {
				robotDirection += 360;
			}

			if (false) {
				std::cout << "theta = " << theta << "\n";
				std::cout << "ang_avg = " << ang_avg << ", theta = " << theta << "\n";
				std::cout << "center: depth = " << averagedDepth << ", dir = " << robotDirection << ", pos=(" << robotPoint.x << ", " << robotPoint.y << ")\n";
			}

			cv::Point objectPoint;
			long double LASER_RATIO = 10.0 / 6.0;
			long double LASER_CONST = 100.0;
			long double objectDistMillis = (long double)averagedDepth / abs(cos(theta * MATH_PI / 180.0));
			bool isValidObjectDist = (objectDistMillis > MILLIS_PER_GRID);
			objectDistMillis += LASER_CONST;
			long double objectDistDiffXMillis = objectDistMillis * sin(robotDirection * MATH_PI / 180.0);
			long double objectDistDiffYMillis = objectDistMillis * cos(robotDirection * MATH_PI / 180.0);
			//long double objectDistDiffXMillis = averagedDepth * tan(theta * MATH_PI / 180);
			//long double objectDistDiffYMillis = averagedDepth;
			objectPoint.x = objectDistDiffXMillis / MILLIS_PER_GRID + robotPoint.x;
			objectPoint.y = objectDistDiffYMillis / MILLIS_PER_GRID + robotPoint.y;

			if (isValidObjectDist) {
				objectPoints.push_back(objectPoint);
			}

			// Extrude shadow
			cv::Point extrudePoint;
			extrudePoint.x = (150 * sin(robotDirection * MATH_PI / 180)) / MILLIS_PER_GRID + objectPoint.x;
			extrudePoint.y = (150 * cos(robotDirection * MATH_PI / 180)) / MILLIS_PER_GRID + objectPoint.y;

			if (averagedDepth != 0) {
				// Draw Interpolated from object to extrude point
				cv::LineIterator it_ex(grid, objectPoint, extrudePoint, 8);
				for (int i = 0; i < it_ex.count; i++, ++it_ex) {
					(*nextProbs.ptr<float>(it_ex.pos().y, it_ex.pos().x)) = 0.52;
				}
			}

			//printf("(_main.cpp:580) <%d> Get Depth Data from Kinect for theta = %d.\n", loop_index, theta);
		}

		

		// Fill Black
		for (int i = 0; i < objectPoints.size(); i++) {
			cv::Point objectPoint = objectPoints[i];
			(*nextProbs.ptr<float>(objectPoints[i].y, objectPoints[i].x)) = 0.55;
		}

		printf("(_main.cpp:593) <%d> Filled Black colors in nextProbs.\n", loop_index);

		// Fill White
		for (int i = 0; i < objectPoints.size(); i++) {
			cv::Point objectPoint = objectPoints[i];
			cv::LineIterator it(grid, robotPoint, objectPoint, 8);

			for (int j = 0; j < it.count - 1; j++, ++it) {
				if (i == 0 || i == objectPoints.size() - 1) {
					(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = 0.5;
				}
				else {
					if ((*nextProbs.ptr<float>(it.pos().y, it.pos().x)) >= 0.54) {
						break;
					}
					if (j != it.count - 1) {
						//(*it)[0] = 0;

						// not object!
						///(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = 0.5;

						(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = 0.45;

						//(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = 0.5 -  ((((4500 - objectDistMillis) / 4500) + (30 - abs(theta)) / 30) / 2)*0.1;

						//updateProbs.push_back(0.8);
					}
					//else {
						//(*it)[0] = 255; 
						//(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = 0.55;
						//(*nextProbs.ptr<float>(it.pos().y, it.pos().x)) = ((((4500-objectDistMillis)/4500)+(30 - abs(theta))/30)/2)*0.1 + 0.5;
						//updateProbs.push_back(1.25);
						//break;
					//}
				}
			}
		}

		// Apply Bayes Filtering
		//bayesFilter(grid, updatePoints, updateProbs);
		if (enableMeasuring) {
			bayesFilter3(grid, nextProbs);
			printf("(_main.cpp:635) <%d> Completed bayesFilter3.\n", loop_index);
		}


		// Grid Clipping
		for (int i = 0; i < GRID_HEIGHT * GRID_WIDTH; i++) {
			if (grid.data[i] <= 3) grid.data[i] = 3;
			else if (grid.data[i] >= 252) grid.data[i] = 252;
		}

		// Generate GridRough by summing corresponding cells in Grid
		gridRough.setTo(cv::Scalar(0));
		
		int ROUGH_FACTOR = ROUGH_MILLIS_PER_GRID / MILLIS_PER_GRID;
		printf("ROUGH_FACTOR = %d\n", ROUGH_FACTOR);
		for (int i = 0; i < GRID_HEIGHT; i++) {
			for (int j = 0; j < GRID_WIDTH; j++) {
				*gridRough.ptr<uint16_t>(i / ROUGH_FACTOR, j / ROUGH_FACTOR) += *grid.ptr<float>(i, j);
			}
		}

		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				*gridRough.ptr<uint16_t>(i, j) /= (ROUGH_FACTOR * ROUGH_FACTOR);
			}
		}

		cBuff.setTo(cv::Scalar(1));		// GREY
		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				if (*gridRough.ptr<uint16_t>(i, j) < 90) {
					*cBuff.ptr<uchar>(i, j) = 0;		// FREE
				} else if (*gridRough.ptr<uint16_t>(i, j) > 160) {
					*cBuff.ptr<uchar>(i, j) = 2;		// OBJ
				}
				//*cBuff.ptr<uchar>(i, j) *= 127;
				*cSpace.ptr<uchar>(i, j) = *cBuff.ptr<uchar>(i, j);
			}
		}

		// Set C-Space
		
		int C_FACTOR = robotSize / (ROUGH_MILLIS_PER_GRID * 2 );
		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				if (*cBuff.ptr<uchar>(i, j) == 2) {
					for (int di = std::max(i - C_FACTOR, 0); di <= std::min(i + C_FACTOR, ROUGH_GRID_HEIGHT); di++) {
						for (int dj = std::max(j - C_FACTOR, 0); dj <= std::min(j + C_FACTOR, ROUGH_GRID_WIDTH); dj++) {
							*cSpace.ptr<uchar>(di, dj) = 2;
						}
					}
				}

			}
		}



		// Convert Grid to Int

		belief_t * gridPtr = grid.ptr<belief_t>(0);
		uchar * gridIntPtr = gridInt.ptr<uchar>(0);
		for (int r = 0; r < grid.rows; r++) {
			for (int c = 0; c < grid.cols; c++) {
				*gridIntPtr = *gridPtr;
				gridIntPtr++;
				gridPtr++;
			}
		}
		cvtColor(gridInt, gridView, CV_GRAY2RGB);
		printf("(_main.cpp:657) <%d> Converted color from gridInt to gridView\n", loop_index);
		for (int i = 0; i < (GRID_HEIGHT * GRID_WIDTH * 3); i++) {
			gridView.data[i] = 255 - gridView.data[i];

			/*if (gridView.data[i] <= 110) {
			gridView.data[i] = 0;
			}
			else if (gridView.data[i] >= 130) {
			gridView.data[i] = 255;
			}
			*/
		}

		// Special Pixels Display //

		// Object Frontier Purple
		uchar * debuggingPixel;
		for (int i = 0; i < objectPoints.size(); i++) {
			cv::Point objectPoint = objectPoints[i];
			debuggingPixel = gridView.ptr<uchar>(objectPoint.y, objectPoint.x);
			*debuggingPixel++ = 0;
			*debuggingPixel++ = 0;
			*debuggingPixel = 255;
		}

		// Player Red
		uchar* playerPixel;
		int playerRadius = (robotSize / MILLIS_PER_GRID) / 2;
		for (int i = -playerRadius; i <= playerRadius; i++) {
			for (int j = -playerRadius; j <= playerRadius; j++) {
				playerPixel = gridView.ptr<uchar>(robotPoint.y + j, robotPoint.x + i);
				*playerPixel++ = 75;
				*playerPixel++ = 75;
				*playerPixel = 255;
			}
		}

		// Direction Green
		uchar* dirPixel;
		if (ang_avg < -135 || ang_avg > 135) {
			dirPixel = gridView.ptr<uchar>(robotPoint.y - 1 - playerRadius, robotPoint.x);
		}
		else if (ang_avg < -45 && ang_avg > -135) {
			dirPixel = gridView.ptr<uchar>(robotPoint.y, robotPoint.x - 1 - playerRadius);
		}
		else if (ang_avg < 45 && ang_avg > -45) {
			dirPixel = gridView.ptr<uchar>(robotPoint.y + 1 + playerRadius, robotPoint.x);
		}
		else if (ang_avg < 135 && ang_avg > 45) {
			dirPixel = gridView.ptr<uchar>(robotPoint.y, robotPoint.x + 1 + playerRadius);
		}
		*dirPixel++ = 100;
		*dirPixel++ = 255;
		*dirPixel = 100;

		// Blue Corner
		uchar * cornerPixels[4] = {
			gridView.ptr<uchar>(0,0),
			gridView.ptr<uchar>(0,gridView.cols - 1),
			gridView.ptr<uchar>(gridView.rows - 1, 0),
			gridView.ptr<uchar>(gridView.rows - 1, gridView.cols - 1)
		};
		for (int i = 0; i < 4; i++) {
			uchar* cornerPixel = cornerPixels[i];
			*cornerPixel++ = 255;
			*cornerPixel++ = 100;
			*cornerPixel = 100;
		}

		printf("(_main.cpp:726) <%d> Setted special pixel colors on GRID_VIEW.\n", loop_index);

		//uchar* objectPixel = gridView.ptr<uchar>(81, 119);
		//*objectPixel++ = 255;
		//*objectPixel++ = 30;
		//*objectPixel = 30;
		imshow("GRID VIEW", gridView);

		//cv::namedWindow("GRID_ROUGH_VIEW", cv::WINDOW_NORMAL);
		//cv::resizeWindow("GRID_ROUGH_VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);
		//imshow("GRID_ROUGH_VIEW", cSpace);


		printf("(_main.cpp:735) <%d> Show GRID_VIEW Image.\n", loop_index);

		int userInput = cv::waitKey(100);
		if (userInput == ' ') {
			enableMeasuring = !enableMeasuring;
		}

		printf("(_main.cpp:742) <%d> Waited for user input (cvWaitKey).\n", loop_index);
		
		loop_index++;
		//printf("===============================================\n");
	}
	return 0;
}
