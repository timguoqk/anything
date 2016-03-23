#pragma once
// Phantom.h: Phantom �N���X�̃C���^�[�t�F�C�X
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_)
#define AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <stdlib.h>		// DDD:vc8�Ή��iglut.h�Ƃ�exit()�̒�`�̋���������j

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
	//�R���X�g���N�^
	Phantom(int number, HDstring name, char* view,
		double pOffset[3], double dOffset[3],
		double p2dScale, double ks, double kb);
	//�f�X�g���N�^
	virtual ~Phantom();

	///////////////////////
	// �N���X���\�b�h
	///////////////////////
	//static void setClassVar(int maxSample, int numData);

	///////////////////////
	// �C���X�^���X���\�b�h
	///////////////////////
	void setPhantomConfiguration(char* view);
	void loadExpSequence(char *fname);
	void initData(int fs, int recordTime, int maxSample, int numData);
	void deleteData();
	void Phantom::setData(int sample, int curTime, double instRate, double TrackingTarget[2]);
	void saveData(int trial, char* fname);

	// �e��v�Z
	void calcConstraintForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime);
	void calcBackHomeForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome);//Hirashima 2009/08/04
	void calcBackTargetForce(const int trial, const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome, double dTargetPos[2], Phantom *p);//Hirashima 2009/08/04
	void calcDPos(double pPos[3], double dPos[3]);
	void calcDVel(double pVel[3], double dVel[3]);
	void getForceFieldMatrix(int trial, double R[18]);
	void calcForceField(int trial, double gVel[6], double pEffectForce[3]);
	void calcErrorCramp(int trial, double rate, double pErrorCramp[3]); // DDD:�G���[�N�����v
	void calcHapticTarget(int trial, double pHapticTarget[3]);// 2009/08/05 �^�[�Q�b�g�ڐG

	void calcDisplayScaleParameter(int trial);
	void calcDTargetRadius();
	void calcDCursorPosVel(int trial);
	void calcDTargetPos(int trial);

	void detectInTarget(int curTime);
	void detectInInitialPos();
	void detectPeakVel(int sample);

	////////////////////////
	// �N���X�ϐ�
	////////////////////////
	//static 

	////////////////////////
	// �C���X�^���X�ϐ�
	////////////////////////
	//-------------------------
	// �ԍ��Ɩ��O
	int number;
	HDstring name;

	//-------------------------
	// open haptics toolkit
	HHD ghHD;
	HHLRC ghHLRC;
	GLuint effect;

	//-------------------------
	// �s��v�Z
	double p2dMatrix[9];//phantom2display
	double d2pMatrix[9];//display2phantom

						//-------------------------
						// �X�P�[���֌W
						//�ȉ���phantom���W�n��ł̎����E�̃X�P�[��
						//double targetDistance;//[mm]
						//double targetForce;//[N]//desktop�ł́A3
	double targetRadius;//[mm]
	double cursorRadius;//[mm]
	double pOffset[3];//[mm]phantom���W�n�̂��̈ʒu�����_�Ƃ݂Ȃ�
	double dOffset[3];//{-0.5,0,0};//<--�������ɕ\��//display��ŃJ�[�\���A�^�[�Q�b�g��`���Ƃ��ɁAdOffset�����v���X���Ă���\���B
					  //�����E����display���E�ւ̃X�P�[���ϊ�
	double p2dScale;
	double p2dScaleForForce;
	double kConnection;
	double dConnection;
	double VisualNoisePosSTD;
	double VisualNoiseVelSTD;

	//-------------------------
	// �F
	double targetForceCol[3];
	double targetMoveCol[3];
	double cursorCol[3];

	//-------------------------
	// ��������
	int nTrials;
	int    *coCond;//constraint condition
	double** ffCond;//force field condition
	double *vmCond;//visuomotor condition
	double *targetAngle;
	double *targetDistance;//[mm]
	double *targetForce;//[N]//desktop�ł́A3
	int    *targetColor;
	double *dErrorCramp; // DDD: �G���[�N�����v�W��
	int *ConnectionType;

	//-------------------------
	//�S�������A���S�΍�
	double ks;
	double kb;

	//phantom���W�n�ł̗́i�T�[�{���[�v���ōX�V�j
	double pForce[3]; double dForce[3];
	double pPos[3]; double dPos[3];
	double pVel[3]; double dVel[3];
	double dCursorPos[3];//VM������������̒l
	double dCursorVel[3];//VM������������̒l

						 //Gimbal
						 //double gAng[3];
	double pTrans[16];
	int button;

	double peakVel;
	bool inDetectPeakVel;
	int velRank;

	//�^�[�Q�b�g���B����
	bool inTarget;
	bool inInitialPos;
	double dTargetRadius;

	double dTargetPos[3];
	int soundTime;//�T�E���h�̕s���������邽�߂ɕK�v
	int soundRestTime;//�T�E���h�s����[ms]

					  //-------------------------
					  // �ۑ��f�[�^
	FILE *fp;
	int fs;
	int recordTime;
	int maxSample;
	int numData;
	double** data;//�񎟌��z��@data[maxSample][numData]
	int* time;//[ms]

	// For backTarget
	int bt_counter = 0;
	double bt_force[3];
};

#endif // !defined(AFX_PHANTOM_H__6C83584A_FBD2_42D8_98D9_2688ABE35BEA__INCLUDED_)
