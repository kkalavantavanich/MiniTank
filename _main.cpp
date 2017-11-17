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
#define RIGHT 3
#define UP 0
#define LEFT 1
#define DOWN 2

/* WebSocket Object used to connect to the GPS. */
using easywsclient::WebSocket;
static WebSocket::pointer wsk = NULL;


double vx = 0;
double vz = 0;
int direction = 0;
double  pointx[4] = { 227.321, -221.07 , -185.694 , 245.306 };
double  pointy[4] = { -169.261 , -173.538 , 168.853, 164.846 };

/* Data Type Object (DTO) of data retrieved from GPS. */
typedef struct {
	int id;
	double x, y, z;
	double angle;
} gps_t;

/* Type of each grid cell (aka. belief grid). */
typedef float belief_t;

/* Regex objects used to match incoming string from GPS WebSocket. */
std::smatch sm;
std::regex re("id: (\\d+)\tpos: (-?\\d+\\.\\d+), (-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\tangle: (-?\\d+\\.\\d+)<br\\/>");

gps_t our_gps;


/* isExpectedStationary :- Used to indicate whether any moving orders have been issued.
 * If isExpectedStationary, then angles and position measurements that are more than threshold will be ignored. 
 * Initialized to false to compensate for initial GPS measurements.
 */
bool isExpectedStationary = false;
const double ANGLE_STATIONARY_THRESHOLD = 7.0;
const double POS_STATIONARY_THRESHOLD = 5.0;

/* Buffers used for averaging measurements retrieved from the GPS.*/
std::vector<double> ang_in_buff;
std::vector<double> x_in_buff;
std::vector<double> y_in_buff;

/* Average measurements retrieved from the GPS.*/
double ang_avg = 0;
double x_avg = 0;
double y_avg = 0;

/* enableMeasuring: use this variable to set whether a measurement is made and updated on the main grid.*/
bool enableMeasuring;

double calculateangle(double x, double y, double ang) {
	double angle = 0;
	double min = 99999999;
	int index;
	for (int i = 0; i < 4; i++) {
		double distance = sqrt(((x - pointx[i])*(x - pointx[i])) + ((y - pointy[i])*(y - pointy[i])));
		if (distance < min) {
			min = distance;
			index = i;
		}
	}
	double angs = atan2(abs(pointy[index] - y), abs(pointx[index] - x)) * 180 / MATH_PI;
	cout << "Angle : " << angs << endl;
	if (x > 0 && y > 0) {
		angle = angs;
	}
	else if (x > 0 && y <= 0) {
		angle = 180 - angs;
	}
	else if (x <= 0 && y > 0) {
		angle = -1 * (angs);
	}
	else if (x <= 0 && y <= 0) {
		angle = -1 * (180 - angs);
	}
	cout << "Index : " << index << " Angle : " << angle << endl;
	//0 Top right 1 Top left 2 Bottom left  3 Bottom right ;
	return angle;
}

/* _movePrototype: Move robot with velocity vx, vz for time millis (and post-delay for 100ms)
* This method is intended to be used inside other moving functions, e.g. turnRight().
*
* @param robot		The robot to control
* @param vx		The velocity in the X-axis (forward direction); in range [-1.0, 1.0]
* @param vz		The velocity in the Z-axis (turning direction); in range [-1.0, 1.0]
* @param millis	The duration of time in milliseconds to move with velocities {vx, vz}.
*/
void _movePrototype(RobotConnector &robot, double vx, double vz, double millis) {

	double vl = vx - vz;
	double vr = vx + vz;

	int velL = (int)(vl*Create_MaxVel);
	int velR = (int)(vr*Create_MaxVel);
	
	isExpectedStationary = false;
	if (!robot.DriveDirect(velL, velR))
		std::cout << "(_main.cpp:91) [ERROR] SetControl Fail\n";

	Sleep(millis);

	if (!robot.DriveDirect(0, 0))
		std::cout << "(_main.cpp:96) [ERROR] SetControl Fail\n";

	Sleep(200);
	isExpectedStationary = true;
}


/* turnRight : Turn the robot right (Clockwise) by the specified angle.
* This function can be fine-tuned to fit physical needs by adjusting the local 'factor' variable.
*
* @param angle		The angle to turn, in degrees. >0
*/
void turnRight(RobotConnector &robot, double angle) {
	double vx, vz;
	vx = 0;
	vz = -0.5;
	double factor = 10.0;		// Conversion factor from angle to turning time.
	long millis = angle * factor;

	_movePrototype(robot, vx, vz, millis);
}

/* turnLeft : Turn the robot left (Counter-clockwise) by the specified angle.
* This function can be fine-tuned to fit physical needs by adjusting the local 'factor' variable.
*
* @param angle		The angle to turn, in degrees. >0
*/
void turnLeft(RobotConnector &robot, double angle) {
	double vx, vz;
	vx = 0;
	vz = 0.5;

	double factor = 10.0;		// Conversion factor from angle to turning time.
	long millis = angle * factor;

	_movePrototype(robot, vx, vz, millis);
}

/* moveForward : Move the robot in the direction it is facing by specified length.
* This function can be fine-tuned to fit physical needs by adjusting the local 'factor' variable.
*
* @param length		The distance to move, in arbitrary units.
*/
void moveForward(RobotConnector &robot, double length) {
	double vx, vz;
	vx = 0.5;
	vz = 0;

	double factor = 40.0;	// Conversion factor from length to turning time.
	long millis = length * factor;

	_movePrototype(robot, vx, vz, millis);
}


/* moveBackward : Move the robot in the opposite direction it is facing by specified length.
* This function can be fine-tuned to fit physical needs by adjusting the local 'factor' variable.
*
* @param length		The distance to move, in arbitrary units.
*/
void moveBackward(RobotConnector &robot, double length) {
	double vx, vz;
	vx = -0.5;
	vz = 0;

	double factor = 40.0;	// Conversion factor from length to turning time.
	long millis = length * factor;

	_movePrototype(robot, vx, vz, millis);
}

void realignToZero(RobotConnector &robot) {
	double deltaAngle = ang_avg;
	if (deltaAngle < 0) {
		turnLeft(robot, -deltaAngle);
	}
	else if (deltaAngle > 0) {
		turnRight(robot, deltaAngle);
	}
}


int checkmap(cv::Mat cSpace, int x, int y) {
	int direc;
	direc = 0;
	/*
	switch (direction)
	{
	case 0:
	if (*cSpace.ptr<uchar>(y + 5, x) == 0 && y + 5 < 80) {
	direc = 0;
	}
	else if (*cSpace.ptr<uchar>(y, x + 5) == 0 && x + 5 < 60) {
	direc = 1;
	}
	else if (*cSpace.ptr<uchar>(y - 5, x) == 0 && y - 5 > 0) {
	direc = 2;
	}
	else {
	direc = 3;
	}
	case 1:
	for (int i = 0; i < 10; i++) {
	if (*cSpace.ptr<uchar>(y, x + 5) == 1 && x + 5 < 60 )  {
	direc = 0;
	}
	else if (*cSpace.ptr<uchar>(y - 5, x) == 1 && y - 5 > 0) {
	direc = 1;
	}
	else if (*cSpace.ptr<uchar>(y, x - 5) == 1 && x - 5 > 0 ) {
	direc = 2;
	}
	else {
	direc = 3;
	}
	}
	case 2:
	for (int i = 0; i < 10; i++) {
	if (*cSpace.ptr<uchar>(y - 5, x) == 1 && y - 5 > 0) {
	direc = 0;
	}
	else if (*cSpace.ptr<uchar>(y, x - 5) == 1 && x - 5 > 0) {
	direc = 1;
	}
	else if (*cSpace.ptr<uchar>(y + 5, x) == 1 && y + 5 < 80) {
	direc = 2;
	}
	else {
	direc = 3;
	}
	}
	case 3:
	for (int i = 0; i < 10; i++) {
	if (*cSpace.ptr<uchar>(y, x - 5) == 1 && x - 5 > 0) {
	direc = 0;
	}
	else if (*cSpace.ptr<uchar>(y + 5, x) == 1 && y + 5 < 80) {
	direc = 1;
	}
	else if (*cSpace.ptr<uchar>(y, x + 5) == 1 && x + 5 < 60) {
	direc = 2;
	}
	else {
	direc = 3;
	}
	}

	}
	*/
	// direc = 0  Turnright
	// direc = 1  Forward
	// direc = 2  Turnleft
	// direc = 3  Backward
	return direc;
}

/*
class BenRobor {
public:
BenRobor(int x, int y, int direction) {
position = make_pair(x, y);
this->direction = direction;

}
pair<int, int> position;
int direction;
bool right(int(&map)[640][480]) {
if (direction == RIGHT) {
if (position.second + 1 > 479) {
return false;
}
if (map[position.second + 1][position.first] != 0) {
return false;
}
return  true;
}
if (direction == UP) {
if (position.first + 1 > 639) {
return false;
}
if (map[position.second][position.first + 1] != 0) {
return false;
}
return  true;
}
if (direction == LEFT) {
if (position.second - 1 <0) {
return false;
}
if (map[position.second - 1][position.first] != 0) {
return false;
}
return  true;
}
if (direction == DOWN) {
if (position.first - 1 < 0) {
return false;
}
if (map[position.second][position.first - 1] != 0) {
return false;
}
return  true;
}
}
bool up(int(&map)[640][480]) {
turnleft();
right(map);
turnright;
}
bool left(int(&map)[640][480]) {
turnleft();
up(map);
turnright;
}
void turnleft() {
direction = (direction + 1) % 4;
}
void turnright() {
direction = (direction + 3) % 4;
}
void setDirection(pair<int, int> p) {
if (p.first - position.first == 1) {
direction = RIGHT;
}
else if (p.second - position.second == -1) {
direction = UP;
}
else if (p.first - position.first == -1) {
direction = LEFT;
}
else if (p.second - position.second == 1) {
direction = DOWN;
}
}
};
BenRobor(robotPoint.x, robotPoint.y, ) ben;
if (!ben.right()) {
if (!ben.up()) {
if (!ben.left()) {
ben.setposition(mempath.pop());
}
else {
ben.turnleft();
mempath.push(ben.position);
}
}
else {
mempath.push(ben.position);
}
}
else {
ben.turnright();
mempath.push(ben.position);
}
*/



bool gpsUpdated = false;
/* handle_message: use this to handle an event-received message by WebSocket.
* The current implementation is to regex into the message and find gps_t which id == 5.
* Additionally, WebSocket will also close if the string 'end' is found in the response message.
*
* @param message		response message received from the server.
*/
void handle_message(const std::string & message)
{
	if (message.find("end") != std::string::npos) {
		wsk->close();
		return;
	}
	std::string s = message;
	std::vector<gps_t> wsDataframe;
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
				cout << "GPS pos x = " << g.x << endl;
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
			printf("(_main.cpp:365) [INFO] found GPS data in stream!\n");

			// If is expected to be stationary, ignore noisy values.
			isExpectedStationary = false;		// Ignore this for now... 
			if (isExpectedStationary) {
				if (abs(our_gps.angle - g.angle) < ANGLE_STATIONARY_THRESHOLD || 360 -  abs(our_gps.angle - g.angle) < ANGLE_STATIONARY_THRESHOLD) {
					our_gps.angle = g.angle;
				}
				if (abs(our_gps.x - g.x) < POS_STATIONARY_THRESHOLD) {
					our_gps.x = g.x;
				}
				if (abs(our_gps.y - g.y) < POS_STATIONARY_THRESHOLD) {
					our_gps.y = g.y;
				}
			}
			else {
				our_gps = g;
			}
			// Moving Average-ish LPF
			ang_avg = (our_gps.angle + ang_avg) / 2;
			x_avg = (our_gps.x + x_avg) / 2;
			y_avg = (our_gps.y + y_avg) / 2;
			gpsUpdated = true;
		}
		s = sm.suffix().str();
	}
}


/* bayesFilter: 1st implementation of a bayesFilter.
* Accepts only perceptual data (from depth sensors).
*
* @param belief		 The main grid Matrix (aka. belief grid)
* @param gridCells	 The cells which is to be updated (by values in probs vector)
* @param probs		 The probability to update the specified gridCells.
*/

// updates in-place! (side-effect)
// receive only perceptual data (sensors)
// TODO: implement action data (motion)
void bayesFilter1(cv::Mat belief, std::vector<cv::Point> gridCells, std::vector<double> probs) {
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
		printf("(_main.cpp:415) [ERROR] in bayesFilter1(): size of gridCells != size of probs.\n");
	}

	if (len == 0) {
		printf("(_main.cpp:419) [ERROR] in bayesFilter1(): InvalidArgument len = 0\n");
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
			printf("(_main.cpp:431) [DEBUG] pos=(%d, %d), *belCellPtr = %d, *cellPtr = %f, prob = %lf, ", p.x, p.y, *beliefCellPtr, *cellPtr, prob);
		}
		(*cellPtr) *= prob;
		if (false) {
			printf("(_main.cpp:435) [DEBUG] afterCellPtr = %f\n", *cellPtr);
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
			printf("(_main.cpp:456) [ERROR] in bayesFilter1: val overflow at pos(%d, %d) val=%d.\n", p.x, p.y, val);
		}
		(*belief.ptr<belief_t>(p.y, p.x)) = val;
	}
}

/* bayesFilter2: 2nd implementation of a bayesFilter.
* Accepts only perceptual data (from depth sensors).
*
* @param belief		 The main grid Matrix (aka. belief grid)
* @param nextProbs	 The probability Matrix to update each grid cells.
*/

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
		printf("(_main.cpp:490) [ERROR] : buff size != nextProbs size");
	}

	float max = 0;
	buffPtr = buff.ptr<float>(0);
	float* nextProbPtr = nextProbs.ptr<float>(0);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			if (false) {
				printf("(_main.cpp:499) [DEBUG] : pos=(%d, %d), buff = %f, nextProb = %f\n", row, col, *buffPtr, *nextProbPtr);
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
				printf("(_main.cpp:526) [ERROR] in bayesFilter2: val overflow at val=%d.\n", val);
			}
			*beliefPtr = val;
			buffPtr++;
			beliefPtr++;
		}
	}

}


/* bayesFilter3: 3rd implementation of a bayesFilter.
* Accepts only perceptual data (from depth sensors).
*
* @param belief		 The main grid Matrix (aka. belief grid)
* @param nextProbs	 The probability Matrix to update each grid cells.
*/
void bayesFilter3(cv::Mat belief, cv::Mat nextProbs) {
	// Create Buffer Matrix which is a float version of belief matrix
	// Values in buffer matrix are in range [0, 1].
	cv::Mat buff;
	buff.create(belief.rows, belief.cols, CV_32FC1);
	assert(buff.rows == belief.rows);
	assert(buff.cols == belief.cols);

	belief_t* beliefPtr = belief.ptr<belief_t>(0);
	float* buffPtr = buff.ptr<float>(0);
	printf("(_main.cpp:553) [INFO] in bayesFilter3: buff.rows = %d, buff.cols = %d\n", buff.rows, buff.cols);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; ++col) {
			*buffPtr = (float)(((float)*beliefPtr) / 255.0);
			buffPtr++;
			beliefPtr++;
		}
	}

	if (nextProbs.rows != buff.rows || nextProbs.cols != buff.cols) {
		printf("(_main.cpp:563) [ERROR] in bayesFilter3: buff size != nextProbs size");
	}

	buffPtr = buff.ptr<float>(0);
	float* nextProbPtr = nextProbs.ptr<float>(0);
	for (int row = 0; row < buff.rows; row++) {
		for (int col = 0; col < buff.cols; col++) {
			if (false) {
				printf("(_main.cpp:571) [DEBUG] pos=(%d, %d), buff = %f, nextProb = %f, ", row, col, *buffPtr, *nextProbPtr);
			}
			(*buffPtr) = (*nextProbPtr)*(*buffPtr) /
				(((1 - (*nextProbPtr))*(1 - (*buffPtr))) + ((*nextProbPtr)*(*buffPtr)));
			(*buffPtr) = (*buffPtr) < 0.08 ? 0.08 : (*buffPtr);
			if (false) {
				printf("(_main.cpp:577) [DEBUG] nextBuff = %f\n", *buffPtr);
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
				printf("(_main.cpp:591) [ERROR] in bayesFilter3: val overflow at val=%d.\n", val);
			}
			*beliefPtr = val;
			buffPtr++;
			beliefPtr++;
		}
	}

}

/* closeSignal: Use this to control the main loop execution. */
bool closeSignal = false;

/****************************************************************\
*                          MAIN METHOD                           *
\****************************************************************/

int main() {
	std::cout << "(_main.cpp:609) [INFO ] Starting...\n";


	// Robot Initialization

	CreateData	robotData;
	RobotConnector	robot;
	if (!robot.Connect(Create_Comport))
	{
		std::cout << "(_main.cpp:618) [FATAL] Cannot connect to robot @" << Create_Comport << "\nExiting...\n";
		return -1;
	}
	else {
		std::cout << "(_main.cpp:622) [INFO ] Connected to robot @" << Create_Comport << "\n";
	}
	robot.DriveDirect(0, 0);
	robot.LEDs(false, false, 0, 255);

	// Kinect Initialization

	KinectConnector kin = KinectConnector();
	if (!kin.Connect()) {
		std::cout << "(_main.cpp:631) [FATAL] Cannot initialize Kinect!\nExiting...\n";
		return -2;
	}
	else {
		std::cout << "(_main.cpp:635) [INFO ] Initialized Kinect\n";
	}


	// BEGIN Grid Variables

	/* There are two types of Grids:
	*  - Smooth Grids (which will also be referred to as simply 'Grid'.
	*  - Rough Grids
	* Smooth Grids are for vision purpose (e.g. The main grid to display).
	* Rough Grids are for motion planning purpose (e.g. cSpace).
	*/

	// Grids Resolution (less = More resolution).
	double MILLIS_PER_GRID = 40;				// Cannot do less than 20 due to grid rough datatype size limit (uint16_t)
	double ROUGH_MILLIS_PER_GRID = 160;

	// World Size (default at 7.2m x 9.6m)
	int WORLD_HEIGHT_MILLIS = 7200;
	int WORLD_WIDTH_MILLIS = 9600;

	// Grid Sizes (Calculated)
	int GRID_HEIGHT = WORLD_HEIGHT_MILLIS / MILLIS_PER_GRID;
	int GRID_WIDTH = WORLD_WIDTH_MILLIS / MILLIS_PER_GRID;
	int ROUGH_GRID_HEIGHT = WORLD_HEIGHT_MILLIS / ROUGH_MILLIS_PER_GRID;
	int ROUGH_GRID_WIDTH = WORLD_WIDTH_MILLIS / ROUGH_MILLIS_PER_GRID;
	std::cout << "(_main.cpp:661) [INFO] GRID SIZE = " << GRID_HEIGHT << "x" << GRID_WIDTH << "\n";

	// Grid View Ports
	int GRID_VIEW_HEIGHT = 480;
	int GRID_VIEW_WIDTH = 640;
	cv::namedWindow("GRID VIEW", cv::WINDOW_NORMAL);
	cv::resizeWindow("GRID VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);
	cv::namedWindow("C-SPACE VIEW", cv::WINDOW_NORMAL);
	cv::resizeWindow("C-SPACE VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);

	// Grid Objects Declarations
	cv::Mat grid;						// Main Grid Object (Float, Greyscale)
	cv::Mat gridView;					// Grid Object use for viewing (RGB Image)
	cv::Mat gridInt;					// Main Grid Objected converted to `uchar` (Greyscale)
	cv::Mat gridRough;					// Rough Grid Object used as intermediate for cSpace Calculation
	cv::Mat cBuff;						// Rough Grid Object used as intermediate for cSpace Calculation
	cv::Mat cSpace;						// Rough Grid Object indicating C-Space (values are either 0 or 1) 
	cv::Mat cSpaceView;					// Rough Grid Object used to view C-Space (RGB Image)
	cv::Mat robotMap;                   // Robotmap for move
	cv::Mat nextProbs;					// Next Probability Matrix for Bayes Filter Calculation. (values in [0.0, 1.0]).
	cv::Mat visitedGrid;				// Rough Grid Object indicating which cells have the robot travelled.

	// Grid Objects Initialization.
	grid = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_32FC1, cv::Scalar(127));
	gridView = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_8UC3);
	gridInt = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_8UC1, cv::Scalar(127));
	gridRough = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_16UC1, cv::Scalar(0));
	cBuff = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC1, cv::Scalar(0));
	cSpace = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC1, cv::Scalar(0));
	cSpaceView = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC3);
	robotMap = cv::Mat(ROUGH_GRID_HEIGHT / 10, ROUGH_GRID_WIDTH / 10, CV_8UC1, cv::Scalar(0));
	nextProbs = cv::Mat(GRID_HEIGHT, GRID_WIDTH, CV_32FC1, cv::Scalar(0.5));
	visitedGrid = cv::Mat(ROUGH_GRID_HEIGHT, ROUGH_GRID_WIDTH, CV_8UC1, cv::Scalar(0));

	// END Grid Variables


	// Robot Variables
	cv::Point robotPoint(GRID_WIDTH / 2, GRID_HEIGHT / 2);	 // Current robot Position in Grid
	const int robotSize = 330;

	// Loop variables
	long double objectDirection = 0.0;

	double KINECT_FOV_H = NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
	double KINECT_FOV_V = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV;

	// Debugging Variables
	int loop_index = 0;				// Count how many loops have passed, use for debugging.

	// Enable Measuring
	enableMeasuring = true;
	
	int objectDistMin = 1000000;
	const int scanCountRst = 8;
	int scanCount = 0;
	const int SCAN_TURNING_DEGREES = 45;

#define ROBOT_STATE_MOVE 0
#define ROBOT_STATE_TURN 1
#define ROBOT_STATE_SCAN 2

	int robotState = ROBOT_STATE_SCAN;
	int findObstrucle = 0;

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

	// Initial Realign

	wsk->send("GET");
	while (ang_avg == 0) {
		printf("_main.cpp:698 GPS get\n");
		wsk->poll(-1);
		wsk->dispatch(handle_message);
	}
	// Remove Averaging effect
	ang_avg *= 2;
	x_avg *= 2;
	y_avg *= 2;

	realignToZero(robot);
	int initialScanCount = 8;

	
	int scansucceed = 0;
	int numturn = 0;
	int movingCounter = 0;

	// Main Loop // 
	
	while (!closeSignal) {

		gpsUpdated = false;
		for (int i = 0; i < 4; i++) {
			printf("_main.cpp:698 GPS get\n");
			wsk->send("GET");
			wsk->poll(-1);
			wsk->dispatch(handle_message);
		}

		printf("(_main.cpp:523) <%d> Read GPS: ang = %lf, x = %lf, y = %lf\n", loop_index, our_gps.angle, our_gps.x, our_gps.y);


		bool step = (loop_index % 7 == 6);
		if (step) {
			if (initialScanCount > 0) {
				initialScanCount--;
				//turnLeft(robot, 45);
			}
		}

		/*if (!findObstrucle) {
		cout << "CP";
		cv::Mat depthImg;
		cv::Mat colorImg;
		cv::Mat indexImg;
		cv::Mat pointImg;
		// Get Data from Kinect to find obstrucle
		kin.GrabData(depthImg, colorImg, indexImg, pointImg);

		if (our_gps.x - 1 > 0 || our_gps.x + 1 < cSpace.cols || our_gps.y - 1 > 0 || our_gps.y + 1 < cSpace.cols) {
		if (depthImg.ptr<int>()) {
		moveForward(robot, 10);
		continue;
		}
		else {
		findObstrucle = 1;
		}

		}
		else {
		findObstrucle = 1;
		}
		}*/
		// Switch robot State //


		/*switch (robotState) {
		case ROBOT_STATE_MOVE:
		enableMeasuring = false;
		if (objectDistMin <= 700) {
		turnLeft(robot, 120);
		robotState = ROBOT_STATE_SCAN;
		}
		break;
		case ROBOT_STATE_TURN:
		enableMeasuring = true;
		if (scanCount >= scanCountRst) {
		scanCount = 0;
		robotState = ROBOT_STATE_MOVE;
		}
		else {
		if (loop_index % 10 == 0) {
		scanCount++;
		}
		robotState = ROBOT_STATE_TURN;
		}
		break;
		case ROBOT_STATE_SCAN :
		enableMeasuring = false;
		}*/
		/*
		int direc;
		double diffang;
		cout << "RobotState : " << robotState << endl;
		switch (robotState) {
		case ROBOT_STATE_MOVE:
			cout << "State move1" << endl;
			if (movingCounter == 0) {
				moveForward(robot, 400);
			}
			else {
				moveForward(robot, 300);
			}

			movingCounter = !movingCounter;
			robotState = ROBOT_STATE_SCAN;
			break;
		case ROBOT_STATE_TURN:
			cout << "State turn" << endl;
			/*diffang =  calculateangle(x_avg, y_avg,ang_avg) - ang_avg ;
			cout << "Diffang : " << diffang  << endl;
			if (diffang < -180) {
			diffang = diffang + 360;
			}
			else if (diffang > 180) {
			diffang = diffang - 360;
			}
			if (diffang < 0) {
			turnRight(robot, -diffang);
			cout << "Turn Right ";
			}
			else {
			turnLeft(robot, diffang);
			cout << "Turn Left ";
			}
			cout << "Diffang : " << diffang << endl;

			//////////////////////////////

			turnLeft(robot, 90);
			cout << "Ang_avg " << ang_avg << endl;
			robotState = ROBOT_STATE_MOVE;
			break;
		case ROBOT_STATE_SCAN:
			cout << "scancount : " << scanCount << endl;
			if (scansucceed) {
				turnLeft(robot, 45);
				scansucceed = 0;
				if (scanCount == 8) {
					scanCount = 0;
					robotState = ROBOT_STATE_TURN;
				}
				break;
			}
		}

		if (robotState != ROBOT_STATE_SCAN) {
			cout << "x_avg : " << x_avg << " y_avg : " << y_avg << " ang : " << ang_avg << endl;
			continue;
		}
		if (numturn == 5) {
			break;
		}
		*/
		printf("_main.cpp:830 finished state case\n");

		// *_in_buff is used as median of 4 values
		/*
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

		x_avg = (x_in_buff[1] + x_in_buff[2]) / 2;
		y_avg = (y_in_buff[1] + y_in_buff[2]) / 2;
		ang_avg = (ang_in_buff[1] + ang_in_buff[2]) / 2;
		*/


		printf("(_main.cpp:948) <%d> Updated ANG_AVG from GPS\n", loop_index);

		nextProbs.setTo(cv::Scalar(0.5));

		cv::Mat depthImg;
		cv::Mat colorImg;
		cv::Mat indexImg;
		cv::Mat pointImg;

		// Get Data from Kinect
		kin.GrabData(depthImg, colorImg, indexImg, pointImg);
		printf("(_main.cpp:959) <%d> Grabbed Data from Kinect.\n", loop_index);
		//if (!robot.ReadData(robotData))
		//cout << "ReadData Fail" << endl;

		// Make Center of ColorImg Red
		for (int theta = -30; theta <= 30; theta++) {
			if (theta == 0) {
				//double fd = (double)theta * colorImg.cols / NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
				long double fd = 2 * NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * tan((long double)theta * MATH_PI / 180.0);
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

		printf("(_main.cpp:1030) <%d> Set color dots for colorImg\n", loop_index);

		imshow("depthImg", depthImg);
		imshow("colorImg", colorImg);

		//printf("(_main.cpp:505) <%d> Showed colorImg and depthImg\n", loop_index);

		robotPoint.x = (x_avg * 10 / MILLIS_PER_GRID) + GRID_WIDTH / 2;
		robotPoint.y = (y_avg * 10 / MILLIS_PER_GRID) + GRID_HEIGHT / 2;
		printf("(_main.cpp:1039) <%d> ang = %lf, x = %lf, y = %lf\n", loop_index, ang_avg, x_avg, y_avg);


		//imshow("indexImg", indexImg);
		//imshow("pointImg", pointImg);
		//std::vector<cv::Point> updatePoints;
		//std::vector<double> updateProbs;
		
		std::vector<cv::Point> objectPoints; // List of Points which are determined in this round of scanning to be an object.
		objectDistMin = 1000000;
		for (int theta = -30; theta <= 30; theta++) {
			// theta valid from approx. -30 to 30
			// pixel_interested = center_pixel + (focal_length * 2 * tan(alpha))
			//double fd = (double)theta * depthImg.rows / NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV;
			long double fd = 2 * NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * tan((long double)theta * MATH_PI / 180.0);
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
			objectDirection = ang_avg + theta;
			if (objectDirection > 180) {
				objectDirection -= 360;
			}
			else if (objectDirection < -180) {
				objectDirection += 360;
			}

			if (false) {
				std::cout << "theta = " << theta << "\n";
				std::cout << "ang_avg = " << ang_avg << ", theta = " << theta << "\n";
				std::cout << "center: depth = " << averagedDepth << ", dir = " << objectDirection << ", pos=(" << robotPoint.x << ", " << robotPoint.y << ")\n";
			}

			cv::Point objectPoint;
			long double LASER_CONST = 100.0;
			long double objectDistMillis = (long double)averagedDepth / abs(cos(theta * MATH_PI / 180.0));
			bool isValidObjectDist = (objectDistMillis > MILLIS_PER_GRID);
			objectDistMillis += LASER_CONST;
			objectDistMin = std::min(objectDistMin, (int)objectDistMillis);
			long double objectDistDiffXMillis = objectDistMillis * sin(objectDirection * MATH_PI / 180.0);
			long double objectDistDiffYMillis = objectDistMillis * cos(objectDirection * MATH_PI / 180.0);
			//long double objectDistDiffXMillis = averagedDepth * tan(theta * MATH_PI / 180);
			//long double objectDistDiffYMillis = averagedDepth;
			objectPoint.x = objectDistDiffXMillis / MILLIS_PER_GRID + robotPoint.x;
			objectPoint.y = objectDistDiffYMillis / MILLIS_PER_GRID + robotPoint.y;

			if (isValidObjectDist) {
				objectPoints.push_back(objectPoint);
			}

			// Extrude shadow Beyond Object
			cv::Point extrudePoint;
			extrudePoint.x = (150 * sin(objectDirection * MATH_PI / 180)) / MILLIS_PER_GRID + objectPoint.x;
			extrudePoint.y = (150 * cos(objectDirection * MATH_PI / 180)) / MILLIS_PER_GRID + objectPoint.y;

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

		printf("(_main.cpp:1032) <%d> Filled Black colors in nextProbs.\n", loop_index);

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
			printf("(_main.cpp:1173) <%d> Completed bayesFilter3.\n", loop_index);
		}

		// Grid Clipping
		for (int i = 0; i < GRID_HEIGHT * GRID_WIDTH; i++) {
			if (grid.data[i] <= 3) grid.data[i] = 3;
			else if (grid.data[i] >= 252) grid.data[i] = 252;
		}

		// Generate GridRough by summing corresponding cells in Grid
		gridRough.setTo(cv::Scalar(0));

		// ROUGH_FACTOR :- the ratio between the resolutions of Smooth Grid and Rough Grid
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

		uchar* boundaryRough;
		cv::Point topLeftBoundaryRough, bottomRightBoundaryRough;
		int GPS_BOUND_OFFSET_X_MM = 2400;
		int GPS_BOUND_OFFSET_Y_MM = 1800;
		topLeftBoundaryRough.x = (ROUGH_GRID_WIDTH / 2) - (GPS_BOUND_OFFSET_X_MM / ROUGH_MILLIS_PER_GRID);
		topLeftBoundaryRough.y = (ROUGH_GRID_HEIGHT / 2) - (GPS_BOUND_OFFSET_Y_MM / ROUGH_MILLIS_PER_GRID);
		bottomRightBoundaryRough.x = (ROUGH_GRID_WIDTH / 2) + (GPS_BOUND_OFFSET_X_MM / ROUGH_MILLIS_PER_GRID);
		bottomRightBoundaryRough.y = (ROUGH_GRID_HEIGHT / 2) + (GPS_BOUND_OFFSET_Y_MM / ROUGH_MILLIS_PER_GRID);


		cBuff.setTo(cv::Scalar(1));		// GREY
		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				if (*gridRough.ptr<uint16_t>(i, j) < 85) {
					*cBuff.ptr<uchar>(i, j) = 0;		// FREE
				}
				else if (*gridRough.ptr<uint16_t>(i, j) > 160) {
					*cBuff.ptr<uchar>(i, j) = 2;		// OBJ
				}
				if ((topLeftBoundaryRough.x == j || bottomRightBoundaryRough.x == j) ||
					(topLeftBoundaryRough.y == i || bottomRightBoundaryRough.y == i)) {
					*cBuff.ptr<uchar>(i, j) = 2;		// OBJ
				}
				//*cBuff.ptr<uchar>(i, j) *= 127;
				*cSpace.ptr<uchar>(i, j) = *cBuff.ptr<uchar>(i, j);
			}
		}

		// Set C-Space

		// C_FACTOR :- the distance which the objects are extruded from its original position.
		// + added QR code size
		int C_FACTOR = (robotSize) / (ROUGH_MILLIS_PER_GRID);

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

		// Update Visited Grid

		*visitedGrid.ptr<uchar>(robotPoint.y / ROUGH_FACTOR, robotPoint.x / ROUGH_FACTOR) = 1;

		// Prepare C-Space for Display
		
		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				*cSpace.ptr<uchar>(i, j) *= 127;
			}
		}

		cvtColor(cSpace, cSpaceView, CV_GRAY2RGB);

		
		// Set Path Blue in C-Space View
		uchar* pathPixel;
		for (int i = 0; i < ROUGH_GRID_HEIGHT; i++) {
			for (int j = 0; j < ROUGH_GRID_WIDTH; j++) {
				if (*visitedGrid.ptr<uchar>(i, j) == 1) {
					pathPixel = cSpaceView.ptr<uchar>(i, j);
					*pathPixel++ = 200;
					*pathPixel++ = 50;
					*pathPixel = 50;
				}
			}
		}

		// Set Player Red in C-Space View.
		uchar* cPlayerPixel = cSpaceView.ptr<uchar>(robotPoint.y / ROUGH_FACTOR, robotPoint.x / ROUGH_FACTOR);
		*cPlayerPixel++ = 75;
		*cPlayerPixel++ = 75;
		*cPlayerPixel = 255;

		imshow("C-SPACE VIEW", cSpaceView);
		

		// MOVING (Kris) //

		// default move forward if empty

		// DISPLAY //

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
		printf("(_main.cpp:1254) <%d> Converted color from gridInt to gridView\n", loop_index);
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

		// Purple Boundary GPS
		uchar* boundaryPixel;
		cv::Point topLeftBoundary, bottomRightBoundary;
		//int GPS_BOUND_OFFSET_X_MM = 2400;
		//int GPS_BOUND_OFFSET_Y_MM = 1800;
		topLeftBoundary.x = (GRID_WIDTH / 2) - (GPS_BOUND_OFFSET_X_MM / MILLIS_PER_GRID);
		topLeftBoundary.y = (GRID_HEIGHT / 2) - (GPS_BOUND_OFFSET_Y_MM / MILLIS_PER_GRID);
		bottomRightBoundary.x = (GRID_WIDTH / 2) + (GPS_BOUND_OFFSET_X_MM / MILLIS_PER_GRID);
		bottomRightBoundary.y = (GRID_HEIGHT / 2) + (GPS_BOUND_OFFSET_Y_MM / MILLIS_PER_GRID);

		// Top & Bottom
		for (int i = topLeftBoundary.x; i <= bottomRightBoundary.x; i++) {
			boundaryPixel = gridView.ptr<uchar>(topLeftBoundary.y, i);
			*boundaryPixel++ = 200;
			*boundaryPixel++ = 75;
			*boundaryPixel = 255;
			boundaryPixel = gridView.ptr<uchar>(bottomRightBoundary.y, i);
			*boundaryPixel++ = 200;
			*boundaryPixel++ = 75;
			*boundaryPixel = 255;
		}

		// Left & Right
		for (int i = topLeftBoundary.y; i <= bottomRightBoundary.y; i++) {
			boundaryPixel = gridView.ptr<uchar>(i, topLeftBoundary.x);
			*boundaryPixel++ = 200;
			*boundaryPixel++ = 75;
			*boundaryPixel = 255;
			boundaryPixel = gridView.ptr<uchar>(i, bottomRightBoundary.x);
			*boundaryPixel++ = 200;
			*boundaryPixel++ = 75;
			*boundaryPixel = 255;
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

		printf("(_main.cpp:1357) <%d> Setted special pixel colors on GRID_VIEW.\n", loop_index);

		//uchar* objectPixel = gridView.ptr<uchar>(81, 119);
		//*objectPixel++ = 255;
		//*objectPixel++ = 30;
		//*objectPixel = 30;
		imshow("GRID VIEW", gridView);

		//cv::namedWindow("GRID_ROUGH_VIEW", cv::WINDOW_NORMAL);
		//cv::resizeWindow("GRID_ROUGH_VIEW", GRID_VIEW_WIDTH, GRID_VIEW_HEIGHT);
		//imshow("GRID_ROUGH_VIEW", cSpace);




		printf("(_main.cpp:1370) <%d> Show GRID_VIEW Image.\n", loop_index);
		
		
		int userInput = cv::waitKey(100);
		
		
		// User-controlled Movement Logic
		if (userInput == ' ') {
			enableMeasuring = !enableMeasuring;
		}
		else if (userInput == 'w') {
			moveForward(robot, 18);
		}
		else if (userInput == 's') {
			moveBackward(robot, 18);
		}
		else if (userInput == 'a') {
			turnLeft(robot, 45);
		}
		else if (userInput == 'd') {
			turnRight(robot, 45);
		}
		else if (userInput == 'r') {
			realignToZero(robot);
		}
		
		printf("(_main.cpp:1391) <%d> Waited for user input (cvWaitKey), received char %d.\n", loop_index, userInput);
		loop_index++;

		if ((loop_index % 8) == 0) {
			scanCount++;
			scansucceed = 1;
		}

		//printf("===============================================\n");


	}
	return 0;
}
