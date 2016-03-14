/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at:
http://dsc.sensable.com

Module Name:

HelloSphere.cpp

Description:

This example demonstrates basic haptic rendering of a shape.

******************************************************************************/
#include <iostream>		// DDD:vc8 correspondence（iostream.h -> iostream)
using namespace std;	// DDD:vc8 correspondence
#include <math.h>
#include <assert.h>
#include <direct.h>
#include <process.h>
#include <cmath>	// for : log, sqrt, sin, cos
#include <sstream>
#include <string>

#if defined(WIN32)
#include <windows.h>
#endif

#include <mmsystem.h>
						//#include <mmsystem.h>
						//timeGetTime()関数を使用するためにはwinmm.libをリンクし mmsystem.hをインクルードする。
						//In order to use the timeGetTime() function, include mmsystem.h link the winmm.lib

#include <GL/gl.h>
						//#include <GL/glui.h>	// DDD:不要と思われるため
						////#include <GL/glui.h>	// DDD: because you think it unecessary 

#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#include "Phantom.h"
#include "helper.h"
#include "AnalogIOManager.h"	// DDD:include analog I/O class 
								


						//***********************
						// 構造体宣言
						// Structure Declaration 
						//***********************
typedef struct
{
	int n;//phantom number
	double pForce_[3];
	double dForce_[3];
	double pPos_[3];
	double dPos_[3];
	int    curTime_;
	bool   inTrial_;
	bool   inSave_;
	int    trial_;
	int    coCond_;
	double ffCond_[NUMFFCOND];
	double vmCond_;
	double targetAngle_;
	double targetDistance_;
	double targetForce_;
	int targetColor_;
	bool inTarget_;
	int velRank_;
	double dCursorPos_[3];
	double dTargetPos_[3];
	double dErrorCramp_;	// DDD:エラークランプ係数 (Error Clamp Coefficient) 
	int button_;
	int ConnectionType; // 0: no connection,

} HapticDisplayState;

typedef struct {
	bool   endTrial_;
} SaveThreadState;

typedef struct {
	int trial;
} PARAM, *lpPARAM;

struct PointMass
{   // Point mass structure, represents a draggable mass. 
#if USE_OHT_HDU_LIB
	hduVector3Dd m_position;
	hduVector3Dd m_velocity;
#endif
	HDdouble m_mass;
	HDdouble m_kStiffness;
	HDdouble m_kDamping;
};
PointMass pointMass;

//*********************************************
// open haptics toolkit related global variables 
//*********************************************
// Shape id for shape we will render haptically.
HLuint gSphereShapeId;

#define CURSOR_SIZE_PIXELS 40 //defalut: 20
static double gCursorScale;
static GLuint gCursorDisplayList = 0;

//****************************
// Function prototypes.
//****************************
void glutDisplay(void);
void glutReshape(int width, int height);
void glutIdle(void);
void glutMenu(int);
void exitHandler(void);
void glutKeyboard(unsigned char key, int x, int y);

void initGL();
void initPHANTOM(int n); // DDD: PHANTOMデバイスのみの初期化 : Phantom initialization of the device only 
void initHL(int n);
void initPhantom();
void initAnalogIO();	// DDD: initialization of the analog I/O data 
bool loadExperiment();	// DDD: Returns false if there is no analog experimental conditions file
void initScene();
void drawSceneHaptics();
void drawSceneGraphics(int Monitor);
void updateWorkspace();

void drawString(const char* string);
void drawPrompts(const char* string, double stringPos[2]);

//コールバック: Callback 
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata);
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata);
void HLCALLBACK computeForceCB0(HDdouble force[3], HLcache *cache, void *userdata);
void HLCALLBACK computeForceCB1(HDdouble force[3], HLcache *cache, void *userdata);
void HLCALLBACK computeForceCB2(HDdouble force[3], HLcache *cache, void *userdata);
void HLCALLBACK computeForceCB3(HDdouble force[3], HLcache *cache, void *userdata);
//thread-safe copy コ callback 
HDCallbackCode HDCALLBACK copyHapticDisplayState(void *pUserData);
HDCallbackCode HDCALLBACK getSaveThreadState(void *pUserData);
HDCallbackCode HDCALLBACK setSaveThreadState(void *pUserData);

void drawCursor(HapticDisplayState *pState, int n);
void drawTarget(HapticDisplayState *pState, int n);

void calcEffectForce(int index, HDdouble _force[3], bool PlanarLock);
void calcSafeForce(int t, HDdouble force[3]);

//display current state (by Yokoi)
void viewState(HapticDisplayState *pState, int para[3]);
void printError(double error, int num);
int mod(int a, int b);
double generateGaussianNoise(double NoiseVar);

//***********************
// Recording Time 
//***********************
int sample = 0;
double TrialLength = 15; // length of trial
const int fs = 1000;//sampling frequency[Hz] //Select from 1000(max), 500, 200, 100
const int recordTime = TrialLength * 1000;//[ms]      //Set recordTime for each experiment
const int maxSample = fs * (recordTime + 1) / 1000; //Don't touch! 
const int waitFrame = (1000 / fs) - 1;//[ms]   //Don't touch!
const int numData = 12;//Don't touch!
int frameFlag = 0;
const int analogIOmaxSample = maxSample*(int)(1000 / fs);//Don't touch! //2009/07/17 Hirashima
bool isAnalogIO = true;

//*************************
//Data storage thread 
//*************************
unsigned int thID;//DWORD thID;
HANDLE hTh;
unsigned __stdcall dataSaveThread(void *);
bool endTrial = false;
//Initialize the structure of the parameters to be passes to the data storage thread
PARAM param;

//***************************
// Frequency of the servo loop [Hz]
//***************************
const int scheduleRate = 1000;//Don't touch!

							  //***********************
							  // Processing of each trial 
							  //***********************
char subject[100];
char set[100];
DWORD globalDwBegin;//the start time of the time measurement[ms]
DWORD globalDwEnd;//timeGetTime[ms]
int curTime;//current time[ms]
int oldTime;//one sample previous to the current time[ms]
int expNum;
int trial = 1;//0 in initializing
bool inTrial = false;//true or false
bool inInitialDetection = false;//true or false
bool inSave = false;//true or false
int initTime;//Detectionに入った時刻: time went into the [ms]
int cumulativeTime;//Detection elapsed time since the beginning of the [ms]
const int holdDuration = 2000;//[ms]task before the start of the hold time 

							  //**********************************************************
							  // trial: after the beginning,changes the target color after a predetermined time
							  //**********************************************************
bool inInitialDetectionForSave = false;//true or false
int initTimeForSave;//Detectionに入った時刻: time went into the[ms]
int cumulativeTimeForSave;//Detection elapsed time since the beginning of the[ms]
int holdDurationForSave = 1000;//[ms]task before the start of the hold time 

							   //**********************************************************
							   // backHome in order to not apply great force suddenly 
							   //**********************************************************
bool backHomeSwitch = false;
bool inBackHome = false;
bool inInitialDetectionForBackHome = false;
int initTimeForBackHome;
int cumulativeTimeForBackHome;

//*********************
//Safety measures 
//*********************
//curTimeがplateauTimeを過ぎると、設定した力がそのまま効力を発揮
const int plateauTime = 1 * 1000;//[ms]
double TWO_PI = 6.2831853071795864769252866;


// EXPERIMENT VARIABLES
int TrialState = 0; // 0=default,1=countdown,2=tracking,3=rest/feedback
int TimeSample = 1; // for recursive calculations
double StateTime = 0; // Length of time in state
double xStateTime = 0, yStateTime = 0;
bool PlanarLock0 = false, PlanarLock1 = false, PlanarLock2 = false, PlanarLock3 = false; // enable planar constraint only during experiment
bool PositionCheck0 = false, PositionCheck1 = false, PositionCheck2 = false, PositionCheck3 = false;
double PlanarLockTime = 0;
bool PlanarLock = false;

// Zeros of target for pseudorandom start
// IF CHANGED, NEED TO CHANGE IN VISUAL NOISE CODE IN CB0
//double xTargetZeros[6] = {0,2.63,3.54,5,6.45,7.36};
//double yTargetZeros[6] = {0,1.38,3.49,5,6.5,8.61};
//double xTargetOffset, yTargetOffset;
double TargetZeros[2][6] = { { 0,2.63,3.54,5,6.45,7.36 },{ 0,1.38,3.49,5,6.5,8.61 } };
double TargetOffset[2] = { 0,0 };
double TrackingTarget[2]; // tracking target
double Error0, Error1, Error2, Error3;
const int NumberSpots = 1;
// Make sure the array sizes are bigger than the number of spots!
double VisualSpotPosX[NumberSpots][4];
double VisualSpotPosY[NumberSpots][4];
double VisualSpotVelX[NumberSpots][4];
double VisualSpotVelY[NumberSpots][4];
double VisualSpotPosSTD[4];
double VisualSpotVelSTD[4];
const double VisualSpotPosSTDMax = 10, VisualSpotPosSTDMin = 10;
const double VisualSpotVelSTDMax = 200, VisualSpotVelSTDMin = 5;
const int NoiseNumber = 10; // how many divisions between max and min of VisualSpotVelSTD
double VisualSpotVelSTDTemplate[NoiseNumber];
double VisualSpotVelSTDStore[NoiseNumber][4];

int NoiseIndex, NoiseArraySize = 1;
int SpotUpdateRate = 400; // spot update rate in ms

						  // Position and velocity checks to lock in plane
double InitPos = 3, InitPosZ = 10, InitVel = 100, InitVelZ = 400; // in mm
double MaxForce = 30;

HDdouble RecordForce[3][4], RampDownForce[3];






double RestTime = 6; // length of rest
int TrainingTrials = 10; // training trials



						 //*********************
						 // Phantom class 
						 //*********************
int nPhantom;
Phantom** phantom;

//*********************
// Gimbal!!! 
//*********************
bool gimbal = true;

//*********************
// Other 
//*********************
int mmm = 0;
int mmmFlag = 0;
bool checkCBThread = false;

/*******************************************************************************
Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main(int argc, char *argv[])
{
	// Initialization of the phantom: 
	initPhantom();

	// Load experiment set-up: 
	//loadExperiment();
	isAnalogIO = loadExperiment();

	// DDD:Initialize analog I/O data 
	// todo:実験条件ファイルがある場合のみ: 
	// todo: if there is experimental conditions file only
	if (isAnalogIO) initAnalogIO();

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	int Height = 1920, Width = 1080;

	// Monitor positions are according to relative vertical spacing (portrait) from top: 4,3,2,1 (4 is at 1280,0)

	//glutInitWindowPosition(1280, Height*3.5);
	glutInitWindowSize(1080, 960);
	glutCreateWindow("Phantom0");

	// Set glut callback functions.
	glutDisplayFunc(glutDisplay);
	glutReshapeFunc(glutReshape);
	glutIdleFunc(glutIdle);

	initGL();

	/*
	glutInitWindowPosition(1280, Height*2.5);
	glutInitWindowSize(1080, 960);
	glutCreateWindow("Phantom1");

	// Set glut callback functions.
	glutDisplayFunc(glutDisplay);
	glutReshapeFunc(glutReshape);
	glutIdleFunc(glutIdle);

	initGL();

	glutInitWindowPosition(1280, Height*1.5);
	glutInitWindowSize(1080, 960);
	glutCreateWindow("Phantom2");



	// Set glut callback functions.
	glutDisplayFunc(glutDisplay);
	glutReshapeFunc(glutReshape);
	glutIdleFunc(glutIdle);

	initGL();

	glutInitWindowPosition(1280, Height*0.5);
	glutInitWindowSize(1080, 960);
	glutCreateWindow("Phantom3");
	*/

	// Set glut callback functions.
	glutDisplayFunc(glutDisplay);
	glutReshapeFunc(glutReshape);
	glutIdleFunc(glutIdle);

	//GLUI_Master.set_glutKeyboardFunc(glutKeyboard);
	glutKeyboardFunc(glutKeyboard);

	glutCreateMenu(glutMenu);
	glutAddMenuEntry("Quit", 0);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	// Provide a cleanup routine for handling application exit.
	atexit(exitHandler);

	//Initialization of the device 
	initScene();

	//Time measurement start 
	timeBeginPeriod(1);
	globalDwBegin = timeGetTime();
	printf("globalDwBegin [ms] = %d,  ", globalDwBegin);

	// DDD:Set the measurement start time to the analog I/0 manager
	if (isAnalogIO) g_analogIOmanager.setBeginTime(globalDwBegin);

	// Data storage thread
	param.trial = 0;
	hTh = (HANDLE)_beginthreadex(NULL, 0, dataSaveThread, &param, 0, &thID);

	glutMainLoop();

	return 0;
}

/////////////////////////////
// Initialization of the phantom
/////////////////////////////
void initPhantom()
{
	// Visual noise
	for (int i = 0; i<NoiseNumber; i++) {
		VisualSpotVelSTDTemplate[i] = VisualSpotVelSTDMin + i*(VisualSpotVelSTDMax - VisualSpotVelSTDMin) / (NoiseNumber - 1);
	}

	// Set the number of phantoms 
	if (1) {
		cout << "Please enter the number of Phantoms" << endl;
		cin >> nPhantom; printf("nPhantom = %d\n", nPhantom);
	}
	else {
		nPhantom = 1;
		//nPhantom = 2;
	}
	// check phantom number
	if (nPhantom<1 || nPhantom>4) {
		printf("nPhantom must be 1-4.\n");
		exit(0);
	}

	// Create an instance of the class Phantom
	phantom = new Phantom*[nPhantom];
	for (int n = 0; n<nPhantom; n++) {
		if (n == 0) {
			HDstring name = "Default PHANToM";//Right
			char* view = "Top";//Top, RightTop, LeftTop
			double pOffset[3] = { 0,450,10 };//[mm] This position on the phantom coordinate system, the origin of the fingertip coordinate system
											 //double dOffset[3] = {0,0,0};    //[mm] display座標系上のこの位置を、指先座標系の原点とする。
			double dOffset[3] = { 0,0,0 };//[mm]
			double p2dScale = 1.0;//phantom scale conversion value from the coordinate system to the display coordinate system (and 1.0 for clarity)
			double ks = 1.0;
			double kb = 0.0001;
			phantom[n] = new Phantom(n, name, view, pOffset, dOffset, p2dScale, ks, kb);
			phantom[n]->initData(fs, recordTime, maxSample, numData);
		}
		else if (n == 1) {
			//HDstring name = "PHANToM1";//the left 
			HDstring name = "phantom 2";//the left 
			char* view = "Top";
			//double pOffset[3] = {120,390,0};//[mm] LeftTop{0,340,130}
			double pOffset[3] = { 0,450,10 };//[mm] Position on the phantom coordinate system 
			double dOffset[3] = { 0,0,0 };//[mm]
			double p2dScale = 1.0;
			double ks = 1.0;
			double kb = 0.0001;
			phantom[n] = new Phantom(n, name, view, pOffset, dOffset, p2dScale, ks, kb);
			phantom[n]->initData(fs, recordTime, maxSample, numData);
		}
		else if (n == 2) {
			HDstring name = "phantom 3";
			char* view = "Top";
			double pOffset[3] = { 0,450,10 };//[mm] Position on the phantom coordinate system。
			double dOffset[3] = { 0,0,0 };//[mm]
			double p2dScale = 1.0;
			double ks = 1.0;
			double kb = 0.0001;
			phantom[n] = new Phantom(n, name, view, pOffset, dOffset, p2dScale, ks, kb);
			phantom[n]->initData(fs, recordTime, maxSample, numData);
		}
		else if (n == 3) {
			HDstring name = "phantom 4";
			char* view = "Top";
			double pOffset[3] = { 0,450,10 };//[mm] Position on the phantom coordinate system
			double dOffset[3] = { 0,0,0 };//[mm]
			double p2dScale = 1.0;
			double ks = 1.0;
			double kb = 0.0001;
			phantom[n] = new Phantom(n, name, view, pOffset, dOffset, p2dScale, ks, kb);
			phantom[n]->initData(fs, recordTime, maxSample, numData);
		}
	}
}

/////////////////////////////
// DDD: Initializing the analog I/O data 
/////////////////////////////
void initAnalogIO()
{
	// initializing the analog I/O boards
	g_analogIOmanager.init();

	// initializing the analog I/o data 
	g_analogIOmanager.initInputData(fs, recordTime, analogIOmaxSample);

	// DDD:analog input start 
	g_analogIOmanager.startMeasurement();
}

/////////////////////////////
// Load the experiment set-up 
/////////////////////////////
bool loadExperiment()
{
	//Read the subject name and experimental conditions 
	cout << "Input subject name." << endl;
	cin >> subject; printf("subject = %s\n", subject);
	char fname[256];
	sprintf(fname, "exptSetup\\exp");
	for (int n = 0; n<nPhantom; n++) {
		phantom[n]->loadExpSequence(fname);
	}


	// Confirmation of n trials 
	// If you are using 2 phantoms, confirm that n trials is the same 
	if (nPhantom == 2) {
		if (phantom[0]->nTrials == phantom[1]->nTrials) {
			//nTrials = phantom[0]->nTrials;
		}
		else {
			printf("nTrials must be same!\nPlease check expPhantom0.csv and expPhantom1.csv\n");
			exit(0);
		}
	}

	if (nPhantom == 3) {
		if ((phantom[0]->nTrials == phantom[1]->nTrials) && (phantom[0]->nTrials == phantom[2]->nTrials)) {
			//nTrials = phantom[0]->nTrials;
		}
		else {
			printf("nTrials must be same!\nPlease check expPhantom0.csv and expPhantom1.csv\n");
			exit(0);
		}
	}

	if (nPhantom == 4) {
		if ((phantom[0]->nTrials == phantom[1]->nTrials) && (phantom[0]->nTrials == phantom[2]->nTrials)
			&& (phantom[0]->nTrials == phantom[3]->nTrials)) {
			//nTrials = phantom[0]->nTrials;
		}
		else {
			printf("nTrials must be same!\nPlease check expPhantom0.csv and expPhantom1.csv\n");
			exit(0);
		}
	}

	// DDD:Reading of analog experimental conditions file 
	if (!g_analogIOmanager.loadExpSequence(fname, phantom[0]->nTrials))
	{
		// Returns false if analog conditions file exits (otherwise true) 
		return false;
	}

	return true;
}

/*******************************************************************************
GLUT callback for redrawing the view.
*******************************************************************************/
void glutDisplay()
{
	//// drawSceneHaptics();//Do not use 

	for (int n = 1; n <= nPhantom; n++) {
		glutSetWindow(n);
		drawSceneGraphics(n);
		glutSwapBuffers();
	}
}

/*******************************************************************************
GLUT callback for reshaping the window.  This is the main place where the
viewing and workspace transforms get initialized.
*******************************************************************************/
void glutReshape(int width, int height)
{
	if (0) {
		//static const double kPI = 3.1415926535897932384626433832795;
		static const double kFovY = 40;

		double nearDist, farDist, aspect;

		glViewport(0, 0, width, height);

		// Compute the viewing parameters based on a fixed fov and viewing
		// a canonical box centered at the origin.

		nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
		farDist = nearDist + 2.0;
		aspect = (double)width / height;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(kFovY, aspect, nearDist, farDist);

		// Camera setting
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// Place the camera down the Z axis looking at the origin.
		gluLookAt(0, 0, nearDist + 1.0, 0, 0, 0, 0, 1, 0);
		// Place the camera down the X axis looking at the origin.
		//gluLookAt(nearDist + 1.0, 0, 0,  0, 0, 0,     0, 1, 0);
		// Place the camera down the Y axis looking at the origin.
		//gluLookAt(0, nearDist + 1.0, 0,   0, 0, 0,     0, 0, -1);

		updateWorkspace();
	}
	else {
		glViewport(0, 0, width, height);

		//Screen size in the real world 
		//Casio projector - the size of the maximum zoom - MUST BE CHANGED 
		double realWidth = 532;//53cm
							   //double realHeight = 430;//43.0cm

							   //When you maximize the window 
							   //Size(pixels) 
		double maxWidth = 1280;
		//double maxHeight = 998;

		//printf("width = %d\n", width);
		//printf("height = %d\n", height);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
			// The window when maximized, the width to be photographed by the projector
			//Set so as to be //realWidth(0.53)
		double w = realWidth*(width / maxWidth) / 2.0;
		double aspect = (double)width / height;
		double h = w / aspect;

		glOrtho(-w, w,
			-h, h,
			-20.0, 20.0);
		//glOrtho(GLdouble left , GLdouble right ,
		//	    GLdouble bottom , GLdouble top , 
		//	    GLdouble near , GLdouble far   );

		// Camera setting
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// Place the camera down the Z axis looking at the origin.
		//gluLookAt(0, 0, 1.0,    0, 0, 0,     0, 1, 0);

		updateWorkspace();

	}

}

/*******************************************************************************
GLUT callback for idle state.  Use this as an opportunity to request a redraw.
Checks for HLAPI errors that have occurred since the last idle check.
*******************************************************************************/
void glutIdle()
{
	HLerror error;

	while (HL_ERROR(error = hlGetError()))
	{
		fprintf(stderr, "HL Error: %s\n", error.errorCode);

#if USE_OHT_HDU_LIB
		if (error.errorCode == HL_DEVICE_ERROR)
		{
			hduPrintError(stderr, &error.errorInfo, "Error during haptic rendering\n");
		}
#endif
	}

	glutPostRedisplay();
}

/******************************************************************************
Popup menu handler.
******************************************************************************/
void glutMenu(int ID)
{
	switch (ID) {
	case 0:
		//exit(0);
		break;
	}
}

/*******************************************************************************
GLUT callback for key presses.
*******************************************************************************/
void glutKeyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'p':
	case 'P':
		//Exit from the current trial 
		inTrial = false;
		inSave = false;
		if (isAnalogIO) {
			g_analogIOmanager.setInTrial(false); // DDD:tells you what is missing from attempts to VAIO 
			g_analogIOmanager.setInSave(false);  // DDD:tells you what is missing from storage mode to VAIO
		}
		sample = 0;
		printf("***************  Trial %d done!!!\n", trial);
		printf("***************  sample %d \n", sample);
		endTrial = true;// <-- dataSaveThreadへ、attempt to end the signal 
		break;
	case 'a':
	case 'A':
		printf("a press!!\n");
		break;
	}
}


/*******************************************************************************
Initializes the scene.  Handles initializing both OpenGL and HL.
*******************************************************************************/
void initScene()
{
	//initGL
	initGL();

	//DDD: initPHANTOM 
	for (int n = 0; n<nPhantom; n++)
	{
		initPHANTOM(n);
	}

	//initHL
	for (int n = 0; n<nPhantom; n++) {
		initHL(n);
	}

}

/*******************************************************************************
Sets up general OpenGL rendering properties: lights, depth buffering, etc.
*******************************************************************************/
void initGL()
{
	static const GLfloat light_model_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	static const GLfloat light0_diffuse[] = { 0.9f, 0.9f, 0.9f, 0.9f };
	static const GLfloat light0_direction[] = { 0.0f, -0.4f, 1.0f, 0.0f };

	// Enable depth buffering for hidden surface removal.
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	// Cull back faces.
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	// Setup other misc features.
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);

	// Setup lighting model.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
	glEnable(GL_LIGHT0);
}

/*******************************************************************************
DDD:Initialize PHANTOM device.
This is moved from initHL() to avoid crashing.
*******************************************************************************/
void initPHANTOM(int n)
{
	HDErrorInfo error;

	phantom[n]->ghHD = hdInitDevice(phantom[n]->name);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
#if USE_OHT_HDU_LIB
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
#endif
		fprintf(stderr, "Press any key to exit");
		getchar();
		exit(-1);
	}
	printf("Found device model: %s / serial number: %s.\n\n",
		hdGetString(HD_DEVICE_MODEL_TYPE), hdGetString(HD_DEVICE_SERIAL_NUMBER));

	// DDD: hlCreateContext() is moved from initHL().
	phantom[n]->ghHLRC = hlCreateContext(phantom[n]->ghHD);
}

/*******************************************************************************
Initialize the HDAPI.  This involves initing a device configuration, enabling
forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void initHL(int n)
{
	//n : current number of phantom

	// DDD: hdInitDevice() is moved to initPHANTOM().
	/*
	HDErrorInfo error;

	phantom[n]->ghHD = hdInitDevice(phantom[n]->name);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
	#if USE_OHT_HDU_LIB
	hduPrintError(stderr, &error, "Failed to initialize haptic device");
	#endif
	fprintf(stderr, "Press any key to exit");
	getchar();
	exit(-1);
	}
	printf("Found device model: %s / serial number: %s.\n\n",
	hdGetString(HD_DEVICE_MODEL_TYPE), hdGetString(HD_DEVICE_SERIAL_NUMBER));
	*/

	hdSetSchedulerRate(scheduleRate);

	// DDD: hlCreateContext() is moved to initPHANTOM().
	// phantom[n]->ghHLRC = hlCreateContext(phantom[n]->ghHD);

	hlMakeCurrent(phantom[n]->ghHLRC);

	// Enable optimization of the viewing parameters when rendering
	// geometry for OpenHaptics.
	hlEnable(HL_HAPTIC_CAMERA_VIEW);

	// Generate id's for the three shapes.
	//gSphereShapeId = hlGenShapes(1);

	hlTouchableFace(HL_FRONT);

	//Effect for the nth Phantom 
	phantom[n]->effect = hlGenEffects(1);
	hlBeginFrame();
	if (n == 0) {
		hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)computeForceCB0, &pointMass);
	}
	else if (n == 1) {
		hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)computeForceCB1, &pointMass);
	}
	else if (n == 2) {
		hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)computeForceCB2, &pointMass);
	}
	else if (n == 3) {
		hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)computeForceCB3, &pointMass);
	}
	hlCallback(HL_EFFECT_START, (HLcallbackProc)startEffectCB, &pointMass);
	hlCallback(HL_EFFECT_STOP, (HLcallbackProc)stopEffectCB, &pointMass);
	hlStartEffect(HL_EFFECT_CALLBACK, phantom[n]->effect);
	hlEndFrame();

}

/*******************************************************************************
This handler is called when the application is exiting.  Deallocates any state
and cleans up.
*******************************************************************************/
void exitHandler()
{
	//time measurement end 
	timeEndPeriod(1);

	// Deallocate the sphere shape id we reserved in initHL.
	hlDeleteShapes(gSphereShapeId, 1);

	if (hTh != NULL) {
		CloseHandle(hTh);
		printf("ハンドルクローズしました\n");
	}

	//the end of the device 
	for (int n = 0; n<nPhantom; n++) {

		//File close 
		if (phantom[n]->fp != NULL) {
			fclose(phantom[n]->fp);
			printf("ファイルクローズしました\n");
		}

		//erasing of the effect 
		hlMakeCurrent(phantom[n]->ghHLRC);
		hlBeginFrame();
		hlStopEffect(phantom[n]->effect);
		hlEndFrame();
		hlDeleteEffects(phantom[n]->effect, 1);

		// Free up the haptic rendering context.
		hlMakeCurrent(NULL);
		if (phantom[n]->ghHLRC != NULL) {
			hlDeleteContext(phantom[n]->ghHLRC);
		}

		/* DDD: デバイスの終了化はまとめて最後に行う
		// Free up the haptic device.
		if (phantom[n]->ghHD != HD_INVALID_HANDLE){
		hdDisableDevice(phantom[n]->ghHD);

		}
		*/
	}

	/* DDD: デバイスの終了化はまとめて最後に行う */
	for (int n = 0; n<nPhantom; n++) {
		// Free up the haptic device.
		if (phantom[n]->ghHD != HD_INVALID_HANDLE) {
			hdDisableDevice(phantom[n]->ghHD);
		}
	}


	//Finally, clear the instance of the Phantom 
	for (int n = 0; n<nPhantom; n++) { // DDD: vc8対応（nにintの定義を追加）
		delete phantom[n];
	}
	delete[] phantom;

	// DDD:アナログ入出力ボードの終了化
	if (isAnalogIO) g_analogIOmanager.destroy();

	// DDD:エラーメッセージ確認用pause
	// system("pause");
}

/*******************************************************************************
Use the current OpenGL viewing transforms to initialize a transform for the
haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void updateWorkspace()
{
	GLdouble modelview[16];
	GLdouble projection[16];
	GLint viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	hlMatrixMode(HL_TOUCHWORKSPACE);
	hlLoadIdentity();

	// Fit haptic workspace to view volume.
	hluFitWorkspace(projection);

	// Compute cursor scale.
	gCursorScale = hluScreenToModelScale(modelview, projection, viewport);
	gCursorScale *= CURSOR_SIZE_PIXELS;

}

/*******************************************************************************
The main routine for displaying the scene.  Gets the latest snapshot of state
from the haptic thread and uses it to display a 3D cursor.
*******************************************************************************/
void drawSceneGraphics(int Monitor)
{
	//************************************
	//グラフィックスレッドのサンプリング
	//************************************
	//printf("openGL curTime = %d\n", state.curTime_);
	// //平均で16.6440678[ms]間隔だった(2008/02/20) ---> 約60[Hz]
	//グラフィックボードとモニタの設定に依存するようだ

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//hlMakeCurrent(ghHLRC);    //これは、hdScheduleSynchronousに対して有効ではない！
	//hdMakeCurrentDevice(ghHD);//これは、hdScheduleSynchronousに対して有効であった！

	//各PHANToMに対して
	//for (int n=0; n<nPhantom; n++){
	HapticDisplayState state;
	state.n = Monitor - 1;
	/* Obtain a thread-safe copy of the current haptic display state. */
	hdScheduleSynchronous(copyHapticDisplayState, &state,
		HD_DEFAULT_SCHEDULER_PRIORITY);

	drawCursor(&state, Monitor);
	drawTarget(&state, Monitor);

	//現在の状態をディスプレイに表示する
	/*
	if(1){
	int para[3] = {1,1,1};
	viewState(&state, para);
	}
	*/
	//}


}

//**************************
// カーソル描画
//**************************
void drawCursor(HapticDisplayState *pState, int Monitor)
{
	int n = pState->n;
	double dCursorPos[3] = { pState->dCursorPos_[0],
		pState->dCursorPos_[1], pState->dCursorPos_[2] };
	/*
	if (0){
	//Visuomotor rotation
	//state.vmCond_がプラスなら反時計回り
	double R[9];
	double rad = pState->vmCond_ * (kPI/180.0);
	R[0] = cos(rad); R[1] =-sin(rad); R[2] = 0;
	R[3] = sin(rad); R[4] = cos(rad); R[5] = 0;
	R[6] = 0;        R[7] = 0;        R[8] = 0;
	mul(R, dCursorPos, dCursorPos);
	}
	*/

	//カーソルが、ターゲットよりも上にかぶるように
	//カーソル半径の分だけ、display座標系の+z方向へ移動
	double dCursorRadius = (phantom[n]->cursorRadius)*(phantom[n]->p2dScale);
	dCursorPos[2] = dCursorRadius;

	// dOffset
	dCursorPos[0] += (phantom[n]->dOffset[0]);
	dCursorPos[1] += (phantom[n]->dOffset[1]);
	dCursorPos[2] += (phantom[n]->dOffset[2]);

	// カーソルの色
	double cursorCol[3] = { 0,0,0 };
	if (pState->inTarget_)	getColor(GREEN, cursorCol);
	else					getColor(WHITE, cursorCol);

	if (pState->button_)	getColor(GREEN, cursorCol);

	//ライティング設定時に glColor3f() を有効にする
	glEnable(GL_COLOR_MATERIAL);
	//OpenGLで球を描く
	glPushMatrix();
	glColor3d(cursorCol[0], cursorCol[1], cursorCol[2]);
	glTranslated(dCursorPos[0], dCursorPos[1], dCursorPos[2]);
	if (TrialState == 0) {
		glutSolidSphere((phantom[n]->targetRadius)*(phantom[n]->p2dScale) + max(1 - (phantom[n]->targetRadius)*(phantom[n]->p2dScale), (phantom[n]->dPos[2]) / 40), 32, 32);
	}
	else {
		glutSolidSphere(dCursorRadius, 32, 32);
	}
	glPopMatrix();

	//------------------------------------
	// 位置
	double stringPos[2] = { 400,700 };
	// 色
	double stringCol[3]; getColor(WHITE, stringCol);
	glColor3d(stringCol[0], stringCol[1], stringCol[2]);


	// FEEDBACK FOR SUBJECTS
	if (TrialState == 0) {
		if (n == 0) {
			if (PositionCheck0) {
				drawPrompts("Please wait...\n", stringPos);
			}
			else {
				if (trial >= phantom[n]->nTrials) {
					drawPrompts("End of experiment\nTHANK YOU!\n", stringPos);
				}
				else {
					drawPrompts("Enter target\n", stringPos);
				}
			}
		}
		if (n == 1) {
			if (PositionCheck1) {
				drawPrompts("Please wait...\n", stringPos);
			}
			else {
				if (trial >= phantom[n]->nTrials) {
					drawPrompts("End of experiment\nTHANK YOU!\n", stringPos);
				}
				else {
					drawPrompts("Enter target\n", stringPos);
				}
			}
		}
		if (n == 2) {
			if (PositionCheck2) {
				drawPrompts("Please wait...\n", stringPos);
			}
			else {
				if (trial >= phantom[n]->nTrials) {
					drawPrompts("End of experiment\nTHANK YOU!\n", stringPos);
				}
				else {
					drawPrompts("Enter target\n", stringPos);
				}
			}
		}
		if (n == 3) {
			if (PositionCheck3) {
				drawPrompts("Please wait...\n", stringPos);
			}
			else {
				if (trial >= phantom[n]->nTrials) {
					drawPrompts("End of experiment\nTHANK YOU!\n", stringPos);
				}
				else {
					drawPrompts("Enter target\n", stringPos);
				}
			}
		}

	}
	if (TrialState == 1) {
		if (StateTime<2) {
			drawPrompts("GET READY...\n", stringPos);
		}
		else if (StateTime >= 2 && StateTime<3) {
			drawPrompts("3\n", stringPos);
		}
		else if (StateTime >= 3 && StateTime<4) {
			drawPrompts("2\n", stringPos);
		}
		else if (StateTime >= 4) {
			drawPrompts("1\n", stringPos);
		}
	}
	// Show error
	if (TrialState == 3) {
		std::ostringstream s;
		if (n == 0) {
			s << "Error: " << floor(100 * Error0) / 100;
		}
		else if (n == 1) {
			s << "Error: " << floor(100 * Error1) / 100;
		}
		else if (n == 2) {
			s << "Error: " << floor(100 * Error2) / 100;
		}
		else if (n == 3) {
			s << "Error: " << floor(100 * Error3) / 100;
		}

		std::string str = s.str();
		double err = floor(100 * Error0) / 100;
		printError(err, trial);
		drawPrompts(str.c_str(), stringPos);
		
	}

}
//prints error to file! 
void printError(double err, int num) {
	FILE *fpExp;
	char fname[256] = "Error.csv";
	fpExp = fopen(fname, "a");
	fprintf(fpExp, "%s,%d \n", err, num);
	fclose(fpExp);


}
//**************************
// ターゲット描画
//**************************
void drawTarget(HapticDisplayState *pState, int Monitor)
{
	int n = pState->n;
	double dTargetPos[3] = { TrackingTarget[0],
		TrackingTarget[1], 0 };

	//トライアル中でない時は、ゼロをセット
	if (TrialState != 2) {
		setZero(dTargetPos);
	}

	//dOffset
	dTargetPos[0] += (phantom[n]->dOffset[0]);
	dTargetPos[1] += (phantom[n]->dOffset[1]);
	dTargetPos[2] += (phantom[n]->dOffset[2]);

	//color
	int colorIndex = pState->targetColor_;
	double targetCol[3] = { 0.5,0.5,0.5 };

	// Target color
	getColor(3, targetCol);//灰色

						   //display上でのターゲットの半径
	double dTargetRadius = (phantom[n]->targetRadius)*(phantom[n]->p2dScale);


	//***** 球 *****
	//ライティング設定時に glColor3f() を有効にする

	//TODO: Figure out target bullshit 
	// draw target(s)
	// XXX: I changed this part
	//if (TrialState == 0 || TrialState == 3) {
		glEnable(GL_COLOR_MATERIAL);
		glPushMatrix();
		glColor3d(targetCol[0], targetCol[1], targetCol[2]);
		glTranslated(dTargetPos[0], dTargetPos[1], dTargetPos[2]);
		glutSolidSphere(InitPos*1.5, 32, 32);
		glPopMatrix();
	//}
	/*
	// noisy target during tracking
	else {
		for (int Spot = 0; Spot<NumberSpots; Spot++) {
			glEnable(GL_COLOR_MATERIAL);
			glPushMatrix();
			glColor3d(targetCol[0], targetCol[1], targetCol[2]);
			glPushMatrix();
			glTranslated(dTargetPos[0] + VisualSpotPosX[Spot][n], dTargetPos[1] + VisualSpotPosY[Spot][n], dTargetPos[2]);
			glutSolidSphere(1.5, 32, 32);
			glPopMatrix();
		}
	}
	*/

}

/*******************************************************************************
The main routine for rendering scene haptics.
*******************************************************************************/
void drawSceneHaptics()
{
	// Start haptic frame.  (Must do this before rendering any haptic shapes.)
	hlBeginFrame();

	// Set material properties for the shapes to be drawn.
	hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);

	// Start a new haptic shape.  Use the feedback buffer to capture OpenGL 
	// geometry for haptic rendering.
	hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, gSphereShapeId);

	// Use OpenGL commands to create geometry.
	//大きい立方体
	glPushMatrix();
	glColor3d(0, 0, 0);
	double len = 2.5;
	glTranslated(0.0, 0.0, -len / 2);
	glutSolidCube(len);
	glPopMatrix();

	// End the shape.
	hlEndShape();

	// End the haptic frame.
	hlEndFrame();
}


/******************************************************************************
Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
	//fprintf(stdout, "Custom effect started\n");
}

/******************************************************************************
Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
	//fprintf(stdout, "Custom effect stopped\n");
}

/******************************************************************************
Servo loop thread callback.  Computes a force effect.
******************************************************************************/
void HLCALLBACK computeForceCB0(HDdouble force[3], HLcache *cache, void *userdata)
{
	//この時点で、forceがゼロであることをチェック
	//printf("force = %f, %f ,%f\n", force[0],force[1],force[2] );

	//---------------------------------------------------
	// 時間の取得
	globalDwEnd = timeGetTime();
	curTime = (int)(globalDwEnd - globalDwBegin);
	//printf("globalDwBegin [ms] = %d,  ", globalDwBegin);
	//printf("globalDwEnd [ms] = %d,  ", globalDwEnd);
	//printf("0: time = %d\n", curTime);//表示は結構重い
	HDdouble beginTime = hdGetSchedulerTimeStamp();


	//カレントデバイスをn番目のPHANToMにセット
	hdMakeCurrentDevice(phantom[0]->ghHD);
	//ファントム座標系表現でゲット
	hdGetDoublev(HD_CURRENT_POSITION, phantom[0]->pPos);//[mm]
	hdGetDoublev(HD_CURRENT_VELOCITY, phantom[0]->pVel);//[mm/s]

														//--------------------------------------------------------
														// Gimbal 2010/11/28
	if (gimbal) {
		//gAngは結局保存しないので、コメントアウト
		//hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, phantom[n]->gAng);//[rad]
		//printf("gAng [deg] = %lf, %lf, %lf \n", phantom[n]->gAng[0]*180/3.14, phantom[n]->gAng[1]*180/3.14, phantom[n]->gAng[2]*180/3.14);

		//pTransは保存するので、有効に。
		hdGetDoublev(HD_CURRENT_TRANSFORM, phantom[0]->pTrans);
	}//-------------------------------------------------------- Gimbal end

	 //ディスプレイ座標系表現に変換
	phantom[0]->calcDPos(phantom[0]->pPos, phantom[0]->dPos);
	phantom[0]->calcDVel(phantom[0]->pVel, phantom[0]->dVel);


	//---------------------------------------------------
	//スレッドチェック
	//--> computeForceCB0とCB1は、どうやら同じスレッドが
	//    交互に実行しているようである。
	if (checkCBThread) {
		if (mmmFlag == 0) {
			mmm = mmm + 1;
			printf("**************************************************************\n");
			printf("0: mmm = %d\n", mmm);
			printf("0:                    time = %d\n", curTime);
			mmmFlag = 1;
		}
	}

	//---------------------------------------------------
	// サーボループ周波数の取得
	HDdouble instRate;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
	//printf("	rate = %f\n", instRate);

	// Increment trial timer
	StateTime += 0.001;
	PlanarLockTime += 0.001;

	// Target at zero by default unless overwritten later
	TrackingTarget[0] = 0; // x-target
	TrackingTarget[1] = 0; // y-target

	TimeSample++;

	//--------------------------------------------------
	// CHECK TRIAL STATES (ONLY FOR PHANTOM0)

	bool PositionCheck;
	//cout << phantom[0]->dPos[0] << ' ' << phantom[0]->dPos[1]hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
	PositionCheck0 = abs(phantom[0]->dPos[0])<InitPos&&abs(phantom[0]->dPos[1])<InitPos&&abs(phantom[0]->dPos[2])<InitPosZ && (abs(phantom[0]->dVel[0]) + abs(phantom[0]->dVel[1]) + abs(phantom[0]->dVel[2]))<InitVel;

	// Drop planar lock if z position out
	for (int n = 0; n<nPhantom; n++) {
		if (abs(phantom[n]->dPos[2])>25 && PlanarLock0 == true && PlanarLock1 == true && PlanarLock2 == true && PlanarLock3 == true) {
			PlanarLock0 = false; PlanarLock1 = false; PlanarLock2 = false; PlanarLock3 = false;
			PlanarLockTime = 0; // reset planar lock time so it doesnt relock immediately
			printf("\n\n\n\n\n\n--->>>>WARNING: PLANAR LOCK DROPPED<<<<---\n");
			TrialState = 3;
			StateTime = 2;
			// Stop saving data
			inSave = false;
			if (isAnalogIO) g_analogIOmanager.setInSave(false);  // DDD:保存モードから抜けたことをAIOに伝える
			endTrial = true;
		}
	}
	// Lock phantom to plane
	if (abs(phantom[0]->dPos[2])<InitPos&&abs(phantom[0]->dVel[2])<InitVelZ&&PlanarLockTime>3) {
		PlanarLock0 = true;
	}
	// State 0: Default (subject moving into position)
	if (TrialState == 0) {

		if (nPhantom == 1) {
			PositionCheck = PositionCheck0;
			PlanarLock = PlanarLock0;
		}
		else if (nPhantom == 2) {
			PositionCheck = PositionCheck0&&PositionCheck1;
			PlanarLock = PlanarLock0;
		}
		else if (nPhantom == 3) {
			PositionCheck = PositionCheck0&&PositionCheck1&&PositionCheck2;
			PlanarLock = PlanarLock0;
		}
		else if (nPhantom == 4) {
			PositionCheck = PositionCheck0&&PositionCheck1&&PositionCheck2&&PositionCheck3;
			PlanarLock = PlanarLock0;
		}


		if (PositionCheck&&PlanarLock) {
			TrialState = 1; // go to countdown
			StateTime = 0; // reset trial timer for countdown
			Error0 = 0; Error1 = 0; Error2 = 0; Error3 = 0; // reset errors
			printf("\n\n\n\nTrial %d\n", trial);

			// Keep visual noise at minimum during training trials
			if (trial <= TrainingTrials) {

				for (int n = 0; n<nPhantom; n++) {
					VisualSpotPosSTD[n] = VisualSpotPosSTDMin;
					VisualSpotVelSTD[n] = VisualSpotVelSTDMin;

					phantom[n]->VisualNoisePosSTD = VisualSpotPosSTD[n];
					phantom[n]->VisualNoiseVelSTD = VisualSpotVelSTD[n];
				}
			}


			// Generate new noise
			if (trial>TrainingTrials&&mod(trial - TrainingTrials - 1, 2) == 0) {

				// Update noise array size
				NoiseArraySize = NoiseArraySize - 1;
				// Reset to noise template if nearly empty
				if (NoiseArraySize == 0) {
					// Restore noise template
					for (int n = 0; n<nPhantom; n++) {
						for (int i = 0; i<NoiseNumber; i++) {
							VisualSpotVelSTDStore[i][n] = VisualSpotVelSTDTemplate[i];
						}
					}

					// Reset noise array size
					NoiseArraySize = NoiseNumber;
				}


				for (int n = 0; n<nPhantom; n++) {
					for (int i = 0; i<NoiseNumber; i++) {
						if (i == NoiseNumber - 1) {
							printf("%3.1f\n ", VisualSpotVelSTDStore[i][n]);
						}
						else {
							printf("%3.1f, ", VisualSpotVelSTDStore[i][n]);
						}
					}
				}



				for (int n = 0; n<nPhantom; n++) {
					VisualSpotPosSTD[n] = VisualSpotPosSTDMin + (VisualSpotPosSTDMax - VisualSpotPosSTDMin)*rand() / ((double)RAND_MAX);

					NoiseIndex = NoiseArraySize*rand() / RAND_MAX;
					VisualSpotVelSTD[n] = VisualSpotVelSTDStore[NoiseIndex][n];
					for (int i = NoiseIndex; i<NoiseNumber; i++) {
						VisualSpotVelSTDStore[i][n] = VisualSpotVelSTDStore[i + 1][n];
					}

					// Update noise
					phantom[n]->VisualNoisePosSTD = VisualSpotPosSTD[n];
					phantom[n]->VisualNoiseVelSTD = VisualSpotVelSTD[n];
				}


			}



			for (int n = 0; n<nPhantom; n++) {
				if (n == nPhantom - 1) {
					printf("Noise%d=%3.1f\n", n, phantom[n]->VisualNoiseVelSTD);
				}
				else {
					printf("Noise%d=%3.1f, ", n, phantom[n]->VisualNoiseVelSTD);
				}
			}


		}
	}

	// State 1: countdown
	if (TrialState == 1) {
		if (StateTime>5) {
			TrialState = 2; // go to tracking state
			StateTime = 0;
			TimeSample = 1;
			sample = 0; // reset data sample index

						// Trigger saving of data
			if (!inSave) {
				inSave = true;//試行に入る
				if (isAnalogIO) g_analogIOmanager.setInSave(true); // DDD: 試行に入ることをAIOに伝える
			}

			// Randomize start of target
			//xTargetOffset = xTargetZeros[sizeof(xTargetZeros)*rand()/(RAND_MAX)];
			//yTargetOffset = yTargetZeros[sizeof(yTargetZeros)*rand()/(RAND_MAX)];
			TargetOffset[0] = TargetZeros[0][6 * rand() / (RAND_MAX)];
			TargetOffset[1] = TargetZeros[1][6 * rand() / (RAND_MAX)];

			xStateTime = TargetOffset[0];
			yStateTime = TargetOffset[1];

			//xStateTime = xTargetOffset;
			//yStateTime = yTargetOffset;

			printf("xOffset=%3.1f,yOffset=%3.1f\n", xStateTime, yStateTime);
			printf("state: tracking...\n");
		}
	}
	// State 2: tracking
	if (TrialState == 2) {

		// Target
		TrackingTarget[0] = 8 * (2 * sin(TWO_PI*0.1*xStateTime) + 5 * sin(TWO_PI*0.3*xStateTime) + 1 * sin(TWO_PI*0.5*xStateTime) + 3 * sin(TWO_PI*0.8*xStateTime)); // x-target
		TrackingTarget[1] = 8 * (1 * sin(TWO_PI*0.2*yStateTime) + 3 * sin(TWO_PI*0.3*yStateTime) + 5 * sin(TWO_PI*0.6*yStateTime) + 3 * sin(TWO_PI*0.8*yStateTime)); // y-target

																																									 // Increment time
		xStateTime += 0.001;
		yStateTime += 0.001;

		Error0 = ((TimeSample - 1)*Error0 + abs(TrackingTarget[0] - phantom[0]->dPos[0]) + abs(TrackingTarget[1] - phantom[0]->dPos[1])) / TimeSample;

		// check trial end
		if (StateTime>TrialLength) {
			TrialState = 3;
			StateTime = 0;
			// Stop saving data
			inSave = false;
			if (isAnalogIO) g_analogIOmanager.setInSave(false);  // DDD:保存モードから抜けたことをAIOに伝える
																 //inTrial = false;	
																 //if (isAnalogIO) g_analogIOmanager.setInTrial(false); // DDD:試行から抜けたことをAIOに伝える
			endTrial = true;

			printf("state: REST\n");

			if (nPhantom == 4) {
				printf("Error0=%3.1f, Error1=%3.1f, Error2=%3.1f, Error3=%3.1f\n", Error0, Error1, Error2, Error3);
			}
		}
	}
	// State 3: rest/feedback
	if (TrialState == 3) {
		if (StateTime>RestTime) {
			TrialState = 0; // return to default after rest/feedback period
			trial++;

			printf("state: rest finished\n");
		}
	}

	// Data saving routine
	if (inSave) {
		//********************
		// メモリに保存
		//********************
		for (int n = 0; n<nPhantom; n++) {
			if (sample<maxSample) {
				phantom[n]->setData(sample, curTime, instRate, TrackingTarget);
			}
			else {
				printf("WARNING: emergency save exit\n");
				// Stop saving data
				inSave = false;
				if (isAnalogIO) g_analogIOmanager.setInSave(false);  // DDD:保存モードから抜けたことをAIOに伝える
				endTrial = true;
			}
		}
		// 更新
		oldTime = curTime;
		sample++;
	}

	// Visual spot update
	if (mod(TimeSample, SpotUpdateRate / NumberSpots) == 0) {// update rate of spots
		for (int n = 0; n<nPhantom; n++) {
			for (int Spot = 0; Spot<NumberSpots - 1; Spot++) {
				// delete spot 0 and shift down
				VisualSpotPosX[Spot][n] = VisualSpotPosX[Spot + 1][n];
				VisualSpotPosY[Spot][n] = VisualSpotPosY[Spot + 1][n];
				VisualSpotVelX[Spot][n] = VisualSpotVelX[Spot + 1][n];
				VisualSpotVelY[Spot][n] = VisualSpotVelY[Spot + 1][n];
			}

			// add new spot at end
			VisualSpotPosX[NumberSpots - 1][n] = generateGaussianNoise(pow(VisualSpotPosSTD[n], 2));
			VisualSpotPosY[NumberSpots - 1][n] = generateGaussianNoise(pow(VisualSpotPosSTD[n], 2));
			VisualSpotVelX[NumberSpots - 1][n] = generateGaussianNoise(pow(VisualSpotVelSTD[n], 2));
			VisualSpotVelY[NumberSpots - 1][n] = generateGaussianNoise(pow(VisualSpotVelSTD[n], 2));
		}
	}
	// update positions of visual spots according to velocity
	for (int n = 0; n<nPhantom; n++) {
		for (int Spot = 0; Spot<NumberSpots; Spot++) {
			VisualSpotPosX[Spot][n] += 0.001*VisualSpotVelX[Spot][n];
			VisualSpotPosY[Spot][n] += 0.001*VisualSpotVelY[Spot][n];
		}
	}


	//---------------------------------------------------
	// 各種のEffectを計算
	int n = 0; //index of phantom
	calcEffectForce(n, force, PlanarLock0);
	//***** （注意）これ以降、forceを追加してはならない *****



	//---------------------------------------------------
	// 状態の判定に必要なパラメータ
	for (n = 0; n<nPhantom; n++) {
		phantom[n]->calcDisplayScaleParameter(trial);
	}

	//---------------------------------
	// タスク終了処理
	if (trial >= phantom[0]->nTrials) {
		printf("------END OF EXPERIMENT-----\n------END OF EXPERIMENT-----\n------END OF EXPERIMENT-----\n");
		exit(0);
	}

	//---------------------------------
	// 最終処理
	// Compute the elasped time within this callback
	if (0) {
		HDdouble endTime = hdGetSchedulerTimeStamp();
		HDdouble callbackTime = endTime - beginTime;
		printf("callbackTime = %f\n", callbackTime);
		printf("endTime = %f\n", endTime);
	}
}

//******************************************
// Second PHANToM
//******************************************
void HLCALLBACK computeForceCB1(HDdouble force[3], HLcache *cache, void *userdata)
{

	//---------------------------------------------------
	// 時間の取得
	DWORD localDwEnd = timeGetTime();
	int localCurTime = (int)(localDwEnd - globalDwBegin);

	//スレッドチェック
	if (checkCBThread) {
		if (mmmFlag == 1) {
			mmm = mmm + 1;
			printf("******************************************\n");
			printf("1: mmm = %d\n", mmm);
			printf("1:                    time = %d\n", localCurTime);
			mmmFlag = 0;
		}
	}

	//---------------------------------------------------
	// サーボループ周波数の取得
	HDdouble instRate;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
	//printf("	rate = %f\n", instRate);

	PositionCheck1 = abs(phantom[1]->dPos[0])<InitPos&&abs(phantom[1]->dPos[1])<InitPos&&abs(phantom[1]->dPos[2])<InitPosZ && (abs(phantom[1]->dVel[0]) + abs(phantom[1]->dVel[1]) + abs(phantom[1]->dVel[2]))<InitVel;

	// Check if phantom can be locked in plane
	if (abs(phantom[1]->dPos[2])<InitPos&&abs(phantom[1]->dVel[2])<InitVelZ&&PlanarLockTime>3) {
		PlanarLock1 = true;
	}


	if (TrialState == 2) {
		Error1 = ((TimeSample - 1)*Error1 + abs(TrackingTarget[0] - phantom[1]->dPos[0]) + abs(TrackingTarget[1] - phantom[1]->dPos[1])) / TimeSample;
	}

	//---------------------------------------------------
	// 各種のEffectを計算
	int n = 1; //index of phantom
	calcEffectForce(n, force, PlanarLock1);
	//***** （注意）これ以降、forceを追加してはならない *****

}

//******************************************
// 3rd PHANToM
//******************************************
void HLCALLBACK computeForceCB2(HDdouble force[3], HLcache *cache, void *userdata)
{

	//---------------------------------------------------
	// 時間の取得
	DWORD localDwEnd = timeGetTime();
	int localCurTime = (int)(localDwEnd - globalDwBegin);

	//スレッドチェック
	if (checkCBThread) {
		if (mmmFlag == 1) {
			mmm = mmm + 1;
			printf("******************************************\n");
			printf("1: mmm = %d\n", mmm);
			printf("1:                    time = %d\n", localCurTime);
			mmmFlag = 0;
		}
	}

	//---------------------------------------------------
	// サーボループ周波数の取得
	HDdouble instRate;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
	//printf("	rate = %f\n", instRate);


	PositionCheck2 = abs(phantom[2]->dPos[0])<InitPos&&abs(phantom[2]->dPos[1])<InitPos&&abs(phantom[2]->dPos[2])<InitPosZ && (abs(phantom[2]->dVel[0]) + abs(phantom[2]->dVel[1]) + abs(phantom[2]->dVel[2]))<InitVel;

	// Check if phantom can be locked in plane
	if (abs(phantom[2]->dPos[2])<InitPos&&abs(phantom[2]->dVel[2])<InitVelZ&&PlanarLockTime>3) {
		PlanarLock2 = true;
	}

	if (TrialState == 2) {
		Error2 = ((TimeSample - 1)*Error2 + abs(TrackingTarget[0] - phantom[2]->dPos[0]) + abs(TrackingTarget[1] - phantom[2]->dPos[1])) / TimeSample;
	}
	//---------------------------------------------------
	// 各種のEffectを計算
	int n = 2; //index of phantom
	calcEffectForce(n, force, PlanarLock2);
	//***** （注意）これ以降、forceを追加してはならない *****

}

//******************************************
// 4th PHANToM
//******************************************
void HLCALLBACK computeForceCB3(HDdouble force[3], HLcache *cache, void *userdata)
{

	//---------------------------------------------------
	// 時間の取得
	DWORD localDwEnd = timeGetTime();
	int localCurTime = (int)(localDwEnd - globalDwBegin);

	//スレッドチェック
	if (checkCBThread) {
		if (mmmFlag == 1) {
			mmm = mmm + 1;
			printf("******************************************\n");
			printf("1: mmm = %d\n", mmm);
			printf("1:                    time = %d\n", localCurTime);
			mmmFlag = 0;
		}
	}

	//---------------------------------------------------
	// サーボループ周波数の取得
	HDdouble instRate;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
	//printf("	rate = %f\n", instRate);


	PositionCheck3 = abs(phantom[3]->dPos[0])<InitPos&&abs(phantom[3]->dPos[1])<InitPos&&abs(phantom[3]->dPos[2])<InitPosZ && (abs(phantom[3]->dVel[0]) + abs(phantom[3]->dVel[1]) + abs(phantom[3]->dVel[2]))<InitVel;

	// Check if phantom can be locked in plane
	if (abs(phantom[3]->dPos[2])<InitPos&&abs(phantom[3]->dVel[2])<InitVelZ&&PlanarLockTime>3) {
		PlanarLock3 = true;
	}

	if (TrialState == 2) {
		Error3 = ((TimeSample - 1)*Error3 + abs(TrackingTarget[0] - phantom[3]->dPos[0]) + abs(TrackingTarget[1] - phantom[3]->dPos[1])) / TimeSample;
	}
	//---------------------------------------------------
	// 各種のEffectを計算
	int n = 3; //index of phantom
	calcEffectForce(n, force, PlanarLock3);
	//***** （注意）これ以降、forceを追加してはならない *****

}




//******************************************
// 各種のEffectを計算
//******************************************
void calcEffectForce(int index, HDdouble _force[3], bool PlanarLock)
{
	//-------------------------------------------------
	// phantom[n]の現在の状態を取得
	int n;
	for (n = 0; n<nPhantom; n++) {
		//カレントデバイスをn番目のPHANToMにセット
		hdMakeCurrentDevice(phantom[n]->ghHD);
		//ファントム座標系表現でゲット
		hdGetDoublev(HD_CURRENT_POSITION, phantom[n]->pPos);//[mm]
		hdGetDoublev(HD_CURRENT_VELOCITY, phantom[n]->pVel);//[mm/s]

															//--------------------------------------------------------
															// Gimbal 2010/11/28
		if (gimbal) {
			//gAngは結局保存しないので、コメントアウト
			//hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, phantom[n]->gAng);//[rad]
			//printf("gAng [deg] = %lf, %lf, %lf \n", phantom[n]->gAng[0]*180/3.14, phantom[n]->gAng[1]*180/3.14, phantom[n]->gAng[2]*180/3.14);

			//pTransは保存するので、有効に。
			hdGetDoublev(HD_CURRENT_TRANSFORM, phantom[n]->pTrans);

			//ボタンも有効に。
			HDint button;
			hdGetIntegerv(HD_CURRENT_BUTTONS, &button);
			phantom[n]->button = button;
			//printf("button = %d\n", phantom[n]->button[0]);
		}//-------------------------------------------------------- Gimbal end

		 //ディスプレイ座標系表現に変換
		phantom[n]->calcDPos(phantom[n]->pPos, phantom[n]->dPos);
		phantom[n]->calcDVel(phantom[n]->pVel, phantom[n]->dVel);

		//状態の判定に必要なパラメータ
		phantom[n]->calcDisplayScaleParameter(trial);//この中でdCursorPos,dCursorVelを計算している

	}

	//-------------------------------------------------
	// After the state aquisition, set the index to n 
	n = index;//index of phantom
	double force[3] = { 0,0,0 };

	// Constrain phantoms to plane (ONLY DURING EXPERIMENT)
	if (PlanarLock) {
		HDdouble pConstraintForce[3] = { 0,0,0 };
		phantom[n]->calcConstraintForce(trial, phantom[n]->dPos, phantom[n]->dVel, pConstraintForce, curTime, plateauTime);
		add(force, pConstraintForce, force);
	}


	// VIEW FROM TOP, CHANGE HERE IF VIEW IS CHANGED
	double p2dMatrix[9], d2pMatrix[9];
	p2dMatrix[0] = 1; p2dMatrix[1] = 0; p2dMatrix[2] = 0;
	p2dMatrix[3] = 0; p2dMatrix[4] = 0; p2dMatrix[5] = -1;
	p2dMatrix[6] = 0; p2dMatrix[7] = 1; p2dMatrix[8] = 0;
	trans(p2dMatrix, d2pMatrix);

	// Connection force
	if (TrialState == 1 || TrialState == 2) {

		HDdouble dConnectionForce[3] = { 0,0,0 }, pConnectionForce[3] = { 0,0,0 };

		// Connection 5: connect all nPhantoms
		if (phantom[n]->ConnectionType[trial] == 5) {

			// other phantom IDs
			int PhantomID[4] = { 0,1,2,3 };

			// find phantoms to connect to
			for (int i = 0; i <= 2; i++) {
				if (i >= n) {
					PhantomID[i] = PhantomID[i + 1];
				}
			}
			// sum all forces from other phantoms
			for (int i = 0; i<nPhantom - 1; i++) {
				dConnectionForce[0] += phantom[n]->kConnection*(phantom[PhantomID[i]]->dPos[0] - phantom[n]->dPos[0]);
				dConnectionForce[1] += phantom[n]->kConnection*(phantom[PhantomID[i]]->dPos[1] - phantom[n]->dPos[1]);
				dConnectionForce[2] += 0;
			}
		}

		// connections 1,2,3,4 only if nPhamtom=4
		if (nPhantom == 4 && phantom[n]->ConnectionType[trial] >= 1 && phantom[n]->ConnectionType[trial] <= 4) {
			int PhantomID2[3];

			// Connection 1: phantoms 0,1,2
			if (phantom[n]->ConnectionType[trial] == 1) {
				PhantomID2[0] = 0; PhantomID2[1] = 1; PhantomID2[2] = 2;

				if (n == 3) {
					PhantomID2[0] = n; PhantomID2[1] = n; PhantomID2[2] = n;
				}
			}
			// Connection 2: phantoms 0,1,3
			if (phantom[n]->ConnectionType[trial] == 2) {
				PhantomID2[0] = 0; PhantomID2[1] = 1; PhantomID2[2] = 3;

				if (n == 2) {
					PhantomID2[0] = n; PhantomID2[1] = n; PhantomID2[2] = n;
				}
			}
			// Connection 3: phantoms 0,2,3
			if (phantom[n]->ConnectionType[trial] == 3) {
				PhantomID2[0] = 0; PhantomID2[1] = 2; PhantomID2[2] = 3;

				if (n == 1) {
					PhantomID2[0] = n; PhantomID2[1] = n; PhantomID2[2] = n;
				}
			}
			// Connection 4: phantoms 1,2,3
			if (phantom[n]->ConnectionType[trial] == 4) {
				PhantomID2[0] = 1; PhantomID2[1] = 2; PhantomID2[2] = 3;

				if (n == 0) {
					PhantomID2[0] = n; PhantomID2[1] = n; PhantomID2[2] = n;
				}
			}

			// find phantoms to connect to
			for (int i = 0; i<2; i++) {
				if (i >= n) {
					PhantomID2[i] = PhantomID2[i + 1];
				}
			}

			// sum all forces from other phantoms
			for (int i = 0; i<2; i++) {
				dConnectionForce[0] += phantom[n]->kConnection*(phantom[PhantomID2[i]]->dPos[0] - phantom[n]->dPos[0]);
				dConnectionForce[1] += phantom[n]->kConnection*(phantom[PhantomID2[i]]->dPos[1] - phantom[n]->dPos[1]);
				dConnectionForce[2] += 0;
			}
		}

		mul(d2pMatrix, dConnectionForce, pConnectionForce);
		add(force, pConnectionForce, force);

		RecordForce[0][n] = pConnectionForce[0];
		RecordForce[1][n] = pConnectionForce[1];
		RecordForce[2][n] = pConnectionForce[2];
	}

	// ramp down in force from tracking to rest
	if (TrialState == 3) {
		RampDownForce[0] = RecordForce[0][n] / (1 + 10 * StateTime);
		RampDownForce[1] = RecordForce[1][n] / (1 + 10 * StateTime);
		RampDownForce[2] = RecordForce[2][n] / (1 + 10 * StateTime);

		if (StateTime<1) {
			add(force, RampDownForce, force);
		}
	}

	//-------------------------------------------------
	// Safety measure 
	calcSafeForce(n, force);

	//-------------------------------------------------
	// Stores the current force to pForce of phantom in the instance
	phantom[n]->pForce[0] = force[0];
	phantom[n]->pForce[1] = force[1];
	phantom[n]->pForce[2] = force[2];

	//Also store dForce 
	mul(phantom[n]->p2dMatrix, force, phantom[n]->dForce);

	//-------------------------------------------------
	// return
	_force[0] = force[0];
	_force[1] = force[1];
	_force[2] = force[2];

}

//******************************************
// 安全対策
//******************************************
void calcSafeForce(int n, HDdouble force[3])
{
	// Make sure max force isnt exceeded
	if (abs(force[0])>MaxForce || abs(force[1])>MaxForce || abs(force[2])>MaxForce) {
		phantom[n]->pForce[0] = 0;
		phantom[n]->pForce[1] = 0;
		phantom[n]->pForce[2] = 0;
		TrialState = 3;
		StateTime = 2;
		PlanarLock = false;
		PlanarLock0 = false; PlanarLock1 = false; PlanarLock2 = false; PlanarLock3 = false;
		printf("TRIAL EXIT: MAXIMUM FORCE REACHED\n");
		// Stop saving data
		inSave = false;
		if (isAnalogIO) g_analogIOmanager.setInSave(false);  // DDD:保存モードから抜けたことをAIOに伝える
		endTrial = true;

	}
	/*
	double kRamp = 0;
	if (t < plateauTime){
	kRamp = pow((double)t, 10)/pow((double)plateauTime, 10); // DDD:vc8対応（powの引数にて明示的キャスト）
	scale(kRamp, force, force);
	//printf("curTime [ms] = %d, kRamp = %lf\n", t, kRamp);

	/*if (t < plateauTime/3.0){
	kRamp = 0.1 * pow(t, 7)/pow((plateauTime/3.0), 7);
	scale(kRamp, force, force);
	}
	else if (t >= plateauTime/3.0 && t < (2.0*plateauTime)/3.0){
	kRamp = 0.1;
	scale(kRamp, force, force);
	}
	else if (t >= (2.0*plateauTime)/3.0){
	kRamp = 0.5 * pow(t-(2.0*plateauTime)/3.0, 7)/pow((plateauTime/3.0), 7);
	kRamp = kRamp + 0.5;
	scale(kRamp, force, force);
	}

	}
	else if (t == plateauTime){
	//printf("***** force plateau *****\n");
	}
	*/

}

/*******************************************************************************
Use this scheduler callback to obtain a thread-safe snapshot of the haptic
device state that we intend on displaying.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyHapticDisplayState(void *pUserData)
{
	HapticDisplayState *pState = (HapticDisplayState *)pUserData;

	int n = pState->n;//printf("*** pState->n = %d\n", pState->n);
	hdMakeCurrentDevice(phantom[n]->ghHD);

	pState->pForce_[0] = phantom[n]->pForce[0];
	pState->pForce_[1] = phantom[n]->pForce[1];
	pState->pForce_[2] = phantom[n]->pForce[2];

	pState->dForce_[0] = phantom[n]->dForce[0];
	pState->dForce_[1] = phantom[n]->dForce[1];
	pState->dForce_[2] = phantom[n]->dForce[2];

	pState->pPos_[0] = phantom[n]->pPos[0];
	pState->pPos_[1] = phantom[n]->pPos[1];
	pState->pPos_[2] = phantom[n]->pPos[2];

	pState->dPos_[0] = phantom[n]->dPos[0];
	pState->dPos_[1] = phantom[n]->dPos[1];
	pState->dPos_[2] = phantom[n]->dPos[2];

	pState->dCursorPos_[0] = phantom[n]->dCursorPos[0];
	pState->dCursorPos_[1] = phantom[n]->dCursorPos[1];
	pState->dCursorPos_[2] = phantom[n]->dCursorPos[2];

	pState->dTargetPos_[0] = phantom[n]->dTargetPos[0];
	pState->dTargetPos_[1] = phantom[n]->dTargetPos[1];
	pState->dTargetPos_[2] = phantom[n]->dTargetPos[2];

	pState->curTime_ = curTime;
	pState->inTrial_ = inTrial;
	pState->inSave_ = inSave;
	pState->trial_ = trial;
	pState->coCond_ = phantom[n]->coCond[trial];
	pState->vmCond_ = phantom[n]->vmCond[trial];
	pState->targetAngle_ = phantom[n]->targetAngle[trial];
	pState->targetDistance_ = phantom[n]->targetDistance[trial];
	pState->targetForce_ = phantom[n]->targetForce[trial];
	pState->targetColor_ = phantom[n]->targetColor[trial];
	pState->inTarget_ = phantom[n]->inTarget;
	pState->velRank_ = phantom[n]->velRank;
	pState->dErrorCramp_ = phantom[n]->dErrorCramp[trial]; // DDD:エラークランプ係数
	pState->button_ = phantom[n]->button;

	//pState->ffCond_  = phantom[n]->ffCond[trial];
	for (int i = 0; i<NUMFFCOND; i++) {
		pState->ffCond_[i] = phantom[n]->ffCond[trial][i];
	}

	return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK getSaveThreadState(void *pUserData)
{
	SaveThreadState *pState = (SaveThreadState *)pUserData;
	pState->endTrial_ = endTrial;
	return HD_CALLBACK_DONE;
}
HDCallbackCode HDCALLBACK setSaveThreadState(void *pUserData)
{
	SaveThreadState *pState = (SaveThreadState *)pUserData;
	endTrial = pState->endTrial_;
	return HD_CALLBACK_DONE;
}

//************************
//  データ保存スレッド
//************************
unsigned __stdcall dataSaveThread(void *lpx)
{
	//構造体のポインタ取得
	//lpPARAM lpParam = (lpPARAM)lpx;
	//構造体内の変数を取得
	//int saveTrial = lpParam->trial;

	char filename[500];
	SaveThreadState saveState;

	while (1) {
		Sleep(10);//10msec

				  // endTrialを検出(thread-safe copy)
		hdScheduleSynchronous(getSaveThreadState, &saveState,
			HD_DEFAULT_SCHEDULER_PRIORITY);

		//******************************************
		// endTrialが検出された場合、データを保存
		//******************************************
		if (saveState.endTrial_) {
			//Save time measurement 
			DWORD iniTime = timeGetTime();

			// 現在の試行番号を取得(thread-safe copy)
			int trial_;
			if (0) {
				//自分で作ったスレッド内でthread-safe copyをする時、
				//コールバック関数の中で、hdGetDoublevが入っていると落ちてしまう。原因不明！
				HapticDisplayState state;
				hdScheduleSynchronous(copyHapticDisplayState, &state,
					HD_DEFAULT_SCHEDULER_PRIORITY);
				trial_ = state.trial_;
			}
			else {
				//代替案として、グローバル変数のtrialを使用する
				//trialはそんなに頻繁に変更されないので、ほぼ安全。
				// As an alternative, to use the trial of the global variable
				//Since // trial is not so much change frequently, almost safety.
				trial_ = trial;
			}

			// 保存ファイル名
			char charTrial[5];
			if (trial_ < 10)		sprintf(charTrial, "000%d", trial_);
			else if (trial_ < 100)	sprintf(charTrial, "00%d", trial_);
			else if (trial_ < 1000)	sprintf(charTrial, "0%d", trial_);
			else					sprintf(charTrial, "%d", trial_);
			//sprintf(filename, "data\\%s\\%sSet%s_%s.csv", subject, subject, set, charTrial);
			sprintf(filename, "data\\%s_%s", subject, charTrial);

			//**************************
			//Save the data for each Phantom 
			//**************************
			for (int n = 0; n<nPhantom; n++) {
				phantom[n]->saveData(trial_, filename);
			}

			//**************************
			// DDD:Saving analog I/O data 
			//**************************
			if (isAnalogIO) g_analogIOmanager.saveData(trial_, filename);

			//int duration = (dataTime[maxSample-1]-dataTime[0]);
			//printf("duration = %d [ms]\n", duration);

			// 保存終了をサーボループスレッドに知らせる
			saveState.endTrial_ = false;
			hdScheduleSynchronous(setSaveThreadState, &saveState,
				HD_DEFAULT_SCHEDULER_PRIORITY);

			// Measurement of storage time 
			DWORD endTime = timeGetTime();
			int saveTime = (int)(endTime - iniTime);
			//printf("saveTime [ms] = %d\n", saveTime);

		}// if (saveState.endTrial_)

	}//while(1)
	return 0;
}

/******************************************************************************
Draws a string using OpenGL. (from ShapeManipulation main.cpp)
******************************************************************************/
void drawString(const char* string)
{
	for (; *string != '\0'; ++string)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *string);
	}
}

/******************************************************************************
Draws string prompts at the bottom of the screen. (from ShapeManipulation main.cpp)
******************************************************************************/
void drawPrompts(const char* string, double stringPos[2])
{
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, 1000, 0, 1000, 0, 1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	//position 
	glRasterPos2f(stringPos[0], stringPos[1]);
	//color 
	glEnable(GL_COLOR_MATERIAL);
	//writing 
	drawString(string);

	//glRasterPos2f(4, 36);
	//drawString("Use mouse to rotate, pan, zoom.");
	//glRasterPos2f(4, 18);
	//drawString("(A) to toggle axis snap, (R) to toggle rotation.");
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
}

/********************************************
Display state at the right top of the screen
********************************************/
void viewState(HapticDisplayState *pState, int para[3])
{
	//para[0]//pPos
	//para[1]//dPos
	//para[2]//pForce

	int n = pState->n;
	double initY = 0;
	double value[3] = { 0,0,0 };

	char pPosChar_0[100];//pPos,dPos,..
	char pPosChar_1[100];//unit 
	char pPosChar_2[100];
	char pPosChar_3[100];

	for (int i = 0; i<3; i++) {
		if (para[i] == 1) {
			if (i == 0) {
				value[0] = pState->pPos_[0] + phantom[n]->pOffset[0];
				value[1] = pState->pPos_[1] + phantom[n]->pOffset[1];
				value[2] = pState->pPos_[2] + phantom[n]->pOffset[2];
				initY = 900;
				sprintf(pPosChar_0, "[pPos+pOffset]");
				sprintf(pPosChar_1, "x:%d [mm] \n", (int)value[0]);
				sprintf(pPosChar_2, "y:%d [mm] \n", (int)value[1]);
				sprintf(pPosChar_3, "z:%d [mm] \n", (int)value[2]);
			}
			else if (i == 1) {
				value[0] = pState->dPos_[0];
				value[1] = pState->dPos_[1];
				value[2] = pState->dPos_[2];
				initY = 700;
				sprintf(pPosChar_0, "[dPos]");
				sprintf(pPosChar_1, "x:%d [mm] \n", (int)value[0]);
				sprintf(pPosChar_2, "y:%d [mm] \n", (int)value[1]);
				sprintf(pPosChar_3, "z:%d [mm] \n", (int)value[2]);
			}
			else if (i == 2) {
				value[0] = pState->dForce_[0];
				value[1] = pState->dForce_[1];
				value[2] = pState->dForce_[2];
				initY = 500;
				sprintf(pPosChar_0, "[dForce]");
				sprintf(pPosChar_1, "x:%3.2lf [N] \n", value[0]);
				sprintf(pPosChar_2, "y:%3.2lf [N] \n", value[1]);
				sprintf(pPosChar_3, "z:%3.2lf [N] \n", value[2]);

			}

			double x[2] = { 850,350 };
			double initX = 0;
			double stringPos[2] = { initX,initY };
			double sep = 25;

			if (n == 0) {
				stringPos[0] = x[0];
			}
			else if (n == 1) {
				stringPos[0] = x[1];
			}

			stringPos[1] -= sep;
			drawPrompts(pPosChar_0, stringPos);

			stringPos[1] -= sep;
			drawPrompts(pPosChar_1, stringPos);

			stringPos[1] -= sep;
			drawPrompts(pPosChar_2, stringPos);

			stringPos[1] -= sep;
			drawPrompts(pPosChar_3, stringPos);
		}
	}
}






/******************************************************************************/

// Modulus function
int mod(int a, int b)
{
	int ret = a % b;
	if (ret < 0)
		ret += b;
	return ret;
}

// White noise function (Box Muller transform)
double generateGaussianNoise(double NoiseVar)
{
	static bool hasSpare = false;
	static double rand1, rand2;

	if (hasSpare)
	{
		hasSpare = false;
		return sqrt(NoiseVar * rand1) * sin(rand2);
	}

	hasSpare = true;

	rand1 = rand() / ((double)RAND_MAX);
	if (rand1 < 1e-100) rand1 = 1e-100;
	rand1 = -2 * log(rand1);
	rand2 = (rand() / ((double)RAND_MAX)) * 2 * TWO_PI;

	return sqrt(NoiseVar * rand1) * cos(rand2);
}