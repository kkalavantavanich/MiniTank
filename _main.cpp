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

std::smatch sm;
std::regex re("id: (\\d+)\tpos: (-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\tangle: (-?\\d+\\.\\d+)<br\\/>");
gps_t our_gps;

void handle_message(const std::string & message)
{
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
		}
		s = sm.suffix().str();
	}
}

int main() {
	std::cout << "starting...\n";


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
	double MILLIS_PER_GRID = 20;
	int GRID_HEIGHT = 7200 / MILLIS_PER_GRID;	// world size in mm
	int GRID_WIDTH = 9600 / MILLIS_PER_GRID;	// world size in mm
	std::cout << "GRID SIZE = " << GRID_HEIGHT << "x" << GRID_WIDTH << "\n";
	int GRID_VIEW_HEIGHT = 480;
	int GRID_VIEW_WIDTH = 640;
	cv::namedWindow("GRID VIEW", cv::WINDOW_NORMAL);
	cv::resizeWindow("GRID VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);
	cv::Mat grid;						// Main Grid Object (Greyscale)
	cv::Mat gridView;					// Grid Object use for viewing (RGB)
	grid.create(GRID_HEIGHT, GRID_WIDTH, CV_8UC1);
	gridView.create(GRID_HEIGHT, GRID_WIDTH, CV_8UC3);
	grid.setTo(127);				// Initialize Grid as 50% Obstacle - 50% Space

									// Get from GPS
	cv::Point robotPoint(GRID_WIDTH / 2, GRID_HEIGHT / 2);			// Current robot Position in Grid
	long double robotDirection = 0.0;     // Robot Direction in range (-180, 180)
	int robotSize = 330;

	cv::Point objectPoint;

	double KINECT_FOV_H = NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
	double KINECT_FOV_V = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV;
	
	// Debugging Variables
	int loop_index = 0;				// Count how many loops have passed, use for debugging.
	//int dt = robotData.distance;

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

	wsk = WebSocket::from_url("ws://192.168.1.59:8080/pose");
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

	// Main Loop // 
	while (true) {
		// Update Position
		wsk->send("GET");
		wsk->poll();
		wsk->dispatch(handle_message);

		
		cv::Mat depthImg;
		cv::Mat colorImg;
		cv::Mat indexImg;
		cv::Mat pointImg;

		// Testing Grid 
		uchar* a = grid.ptr<uchar>();
		a[10 * GRID_WIDTH + 10] = 255;
		a[11 * GRID_WIDTH + 11] = 127;
		*(grid.ptr<uchar>(12, 12)) = 255;

		// Get Data from Kinect
		kin.GrabData(depthImg, colorImg, indexImg, pointImg);
		//if (!robot.ReadData(robotData))
		//cout << "ReadData Fail" << endl;

		// Make Center of ColorImg Red
		for (int theta = -30; theta <= 30; theta++) {
			if (theta == 0) {
				double fd = (double)theta * colorImg.cols / 62;
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
				double fd = (double)theta * colorImg.cols / 62;
				uchar* colorImgCenter = colorImg.ptr<uchar>(479, 639 + fd);
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				colorImgCenter = colorImg.ptr<uchar>(480, 639 + fd);
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 255;
				*colorImgCenter++ = 0;
				*colorImgCenter++ = 0;
			}
		}

		imshow("depthImg", depthImg);
		imshow("colorImg", colorImg);
		//imshow("indexImg", indexImg);
		//imshow("pointImg", pointImg);
		for (int theta = -30; theta <= 30; theta++) {
			// theta valid from approx. -30 to 30
			double fd = (double)theta * depthImg.rows / 62;
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
				if (averagedPoints[i] > 500 && averagedPoints[i] < 4500) {
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
			robotDirection = our_gps.angle + theta;
			if (robotDirection > 180) {
				robotDirection -= 360;
			}
			else if (robotDirection < -180) {
				robotDirection += 360;
			}
			robotPoint.x = (our_gps.x * 10 / MILLIS_PER_GRID) + GRID_WIDTH / 2;
			robotPoint.y = (our_gps.y * 10 / MILLIS_PER_GRID) + GRID_HEIGHT / 2;
			
			if (false) {
				std::cout << "theta = " << theta << "\n";
				std::cout << "our_gps.angle = " << our_gps.angle << ", theta = " << theta << "\n";
				std::cout << "center: depth = " << averagedDepth << ", dir = " << robotDirection << ", pos=(" << robotPoint.x << ", " << robotPoint.y << ")\n";
			}
			long double objectDistMillis = averagedDepth;
			long double objectDistDiffXMillis = objectDistMillis * sin(robotDirection * MATH_PI / 180);
			long double objectDistDiffYMillis = objectDistMillis * cos(robotDirection * MATH_PI / 180);
			objectPoint.x = objectDistDiffXMillis / MILLIS_PER_GRID + robotPoint.x;
			objectPoint.y = objectDistDiffYMillis / MILLIS_PER_GRID + robotPoint.y;

			// Draw Interpolated Line to object
			if (averagedDepth != 0) {
				cv::LineIterator it(grid, robotPoint, objectPoint, 8);
				for (int i = 0; i < it.count; i++, ++it) {
					(*it)[0] = 0;
					if (i == it.count - 1) {
						(*it)[0] = 255;
					}
				}
			}
		}

		cvtColor(grid, gridView, CV_GRAY2RGB);

		for (int i = 0; i < (GRID_HEIGHT * GRID_WIDTH * 3); i++) {
			gridView.data[i] = 255 - gridView.data[i];
		}
		uchar* playerPixel = gridView.ptr<uchar>(robotPoint.y, robotPoint.x);
		*playerPixel++ = 100;
		*playerPixel++ = 100;
		*playerPixel = 255;
		//uchar* objectPixel = gridView.ptr<uchar>(objectPoint.y, objectPoint.x);
		//*objectPixel++ = 255;
		//*objectPixel++ = 100;
		//*objectPixel = 100;
		imshow("GRID VIEW", gridView);
		cv::waitKey(10);

		loop_index++;
	}
	return 0;
}