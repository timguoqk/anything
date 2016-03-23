#pragma once
// Phantom.h: Phantom クラスのインターフェイス
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_)
#define AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdlib.h>		// DDD:vc8対応（glut.hとのexit()の定義の競合を回避）

#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

// Declare include header for using OHT HDU libraries.
#ifndef USE_OHT_HDU_LIB
// By using OHT version up to 3.1, some function ( e.g. hduPrintError() )
// in the HDU library occurs function linking error. 
#define USE_OHT_HDU_LIB 0
#endif

#if USE_OHT_HDU_LIB
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#else
#include <stdio.h>	 // for using : FILE
#endif

#include <HD/hd.h>
#include <HL/hl.h>
#include <HLU/hlu.h>


#define NUMFFCOND 8

enum {
	FAST = 1,
	SLOW = 2,
	GOOD = 3
};

class Phantom
{
public:
	//コンストラクタ
	Phantom(int number, HDstring name, char* view,
		double pOffset[3], double dOffset[3],
		double p2dScale, double ks, double kb);
	//デストラクタ
	virtual ~Phantom();

	///////////////////////
	// クラスメソッド
	///////////////////////
	//static void setClassVar(int maxSample, int numData);

	///////////////////////
	// インスタンスメソッド
	///////////////////////
	void setPhantomConfiguration(char* view);
	void loadExpSequence(char *fname);
	void initData(int fs, int recordTime, int maxSample, int numData);
	void deleteData();
	void Phantom::setData(int sample, int curTime, double instRate, double TrackingTarget[2]);
	void saveData(int trial, char* fname);

	// 各種計算
	void calcConstraintForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime);
	void calcBackHomeForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome);//Hirashima 2009/08/04
	void calcBackTargetForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome, double dTargetPos[2], Phantom *p);//Hirashima 2009/08/04
	void calcDPos(double pPos[3], double dPos[3]);
	void calcDVel(double pVel[3], double dVel[3]);
	void getForceFieldMatrix(int trial, double R[18]);
	void calcForceField(int trial, double gVel[6], double pEffectForce[3]);
	void calcErrorCramp(int trial, double rate, double pErrorCramp[3]); // DDD:エラークランプ
	void calcHapticTarget(int trial, double pHapticTarget[3]);// 2009/08/05 ターゲット接触

	void calcDisplayScaleParameter(int trial);
	void calcDTargetRadius();
	void calcDCursorPosVel(int trial);
	void calcDTargetPos(int trial);

	void detectInTarget(int curTime);
	void detectInInitialPos();
	void detectPeakVel(int sample);

	////////////////////////
	// クラス変数
	////////////////////////
	//static 

	////////////////////////
	// インスタンス変数
	////////////////////////
	//-------------------------
	// 番号と名前
	int number;
	HDstring name;

	//-------------------------
	// open haptics toolkit
	HHD ghHD;
	HHLRC ghHLRC;
	GLuint effect;

	//-------------------------
	// 行列計算
	double p2dMatrix[9];//phantom2display
	double d2pMatrix[9];//display2phantom

						//-------------------------
						// スケール関係
						//以下はphantom座標系上での実世界のスケール
						//double targetDistance;//[mm]
						//double targetForce;//[N]//desktopでは、3
	double targetRadius;//[mm]
	double cursorRadius;//[mm]
	double pOffset[3];//[mm]phantom座標系のこの位置を原点とみなす
	double dOffset[3];//{-0.5,0,0};//<--少し左に表示//display上でカーソル、ターゲットを描くときに、dOffsetだけプラスしてから表示。
					  //実世界からdisplay世界へのスケール変換
	double p2dScale;
	double p2dScaleForForce;
	double kConnection;
	double dConnection;
	double VisualNoisePosSTD;
	double VisualNoiseVelSTD;

	//-------------------------
	// 色
	double targetForceCol[3];
	double targetMoveCol[3];
	double cursorCol[3];

	//-------------------------
	// 実験条件
	int nTrials;
	int    *coCond;//constraint condition
	double** ffCond;//force field condition
	double *vmCond;//visuomotor condition
	double *targetAngle;
	double *targetDistance;//[mm]
	double *targetForce;//[N]//desktopでは、3
	int    *targetColor;
	double *dErrorCramp; // DDD: エラークランプ係数
	int *ConnectionType;

	//-------------------------
	//拘束条件、安全対策
	double ks;
	double kb;

	//phantom座標系での力（サーボループ内で更新）
	double pForce[3]; double dForce[3];
	double pPos[3]; double dPos[3];
	double pVel[3]; double dVel[3];
	double dCursorPos[3];//VMがかかった後の値
	double dCursorVel[3];//VMがかかった後の値

						 //Gimbal
						 //double gAng[3];
	double pTrans[16];
	int button;

	double peakVel;
	bool inDetectPeakVel;
	int velRank;

	//ターゲット到達判定
	bool inTarget;
	bool inInitialPos;
	double dTargetRadius;

	double dTargetPos[3];
	int soundTime;//サウンドの不応期をつくるために必要
	int soundRestTime;//サウンド不応期[ms]

					  //-------------------------
					  // 保存データ
	FILE *fp;
	int fs;
	int recordTime;
	int maxSample;
	int numData;
	double** data;//二次元配列　data[maxSample][numData]
	int* time;//[ms]

	// For backTarget
	int bt_counter = 0;
	double bt_force[3];
};

#endif // !defined(AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_)
