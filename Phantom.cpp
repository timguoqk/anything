// Phantom.cpp: Phantom �N���X�̃C���v�������e�[�V����
//
//////////////////////////////////////////////////////////////////////

#include "Phantom.h"
#include "helper.h"

#include <cmath>	// for : log, sqrt, sin, cos


//////////////////////////////////////////////////////////////////////
// �R���X�g���N�^
//////////////////////////////////////////////////////////////////////
Phantom::Phantom(int _number, HDstring _name, char* _view,
	double _pOffset[3], double _dOffset[3],
	double _p2dScale, double _ks, double _kb)
{
	number = _number;
	name = _name;
	printf("PHANToM[%d] = %s\n", number, name);

	// �팱�҂ɑ΂���t�@���g���̎p�����Z�b�g
	setPhantomConfiguration(_view);

	//-------------------------
	// open haptics toolkit
	ghHD = HD_INVALID_HANDLE;
	ghHLRC = 0;
	GLuint effect = 0;

	//-------------------------
	// Scale Relationship 
	//Real - world scale on the phantom coordinate system: 
	targetRadius = 5;//[mm]
	cursorRadius = 3;//[mm]
					 //pOffset[mm]: fingertip origin of this position of the phantom coordinate system
	pOffset[0] = _pOffset[0];
	pOffset[1] = _pOffset[1];
	pOffset[2] = _pOffset[2];
	//dOffset[mm]: Cursor on the display, when you draw a target, display from plus only dOffset�B
	dOffset[0] = _dOffset[0];
	dOffset[1] = _dOffset[1];
	dOffset[2] = _dOffset[2];

	//�����E����display���E�ւ̃X�P�[���ϊ�
	p2dScale = _p2dScale;
	//const double p2dScaleForForce = (targetDistance * p2dScale)/targetForce;
	//display��ł̃^�[�Q�b�g�̋����������ɂȂ�悤�ɃZ�b�g

	//-------------------------
	// �F
	memcpy(cursorCol, white, sizeof(white));

	//-------------------------
	// �������� --> loadExpSequence�Ɉڍs�ς�
	//int    *coCond;//constraint condition
	//double *ffCond;//force field condition
	//double *vmCond;//visuomotor condition
	//double *targetAngle;

	//-------------------------
	// PLANAR CONSTRAINT STRENGTH
	ks = 0.8;//stiffness [N/mm]//desktop�Ȃ�0.5�`1.0 //premium�Ȃ�2.0��OK
	kb = 0.0001;
	//damping [N/(mm/s)] 0.0005���Ə����S����������B
	//0.0001���ƂقƂ�ǂ킩��Ȃ�
	// Connection strengths
	kConnection = 0.1;
	dConnection = 0.0001;



	//Force at phantom coordinate system(updated in the servo loop�j
	setZero(pForce);
	setZero(pPos);
	setZero(pVel);
	setZero(dPos);
	setZero(dVel);
	setZero(dCursorPos);
	setZero(dCursorVel);

	peakVel = 0;
	inDetectPeakVel = false;
	velRank = GOOD;

	inTarget = false;
	dTargetRadius = 0;
	setZero(dTargetPos);

	soundTime = 0;//�T�E���h�̕s���������邽�߂ɕK�v
	soundRestTime = 1000;//�T�E���h�s����[ms]

						 //Gimbal�ϐ�������
						 //setZero(gAng);
	button = 0;
	for (int i = 0; i<16; i++) {
		pTrans[i] = 0.0;
	}

	fp = NULL; // DDD:��x�����B���Ȃ������ꍇ�ł�����I�������邽��
}
//////////////////////////////////////////////////////////////////////
// �f�X�g���N�^
//////////////////////////////////////////////////////////////////////
Phantom::~Phantom()
{
	deleteData();
}

//////////////////////////////////////////////////////////////////////
// �N���X���\�b�h
//////////////////////////////////////////////////////////////////////
/*
static void setClassVar(int _maxSample, int _numData)
{
Phantom::maxSample = _maxSample;
Phantom::numData   = _numData;
if (doneSetClassVar){
printf("doneSetClassVar = TRUE");
}
else{
printf("doneSetClassVar = FALSE");
doneSetClassVar = ture;
}
}
*/

//////////////////////////////////////////////////////////////////////
// �C���X�^���X���\�b�h
//////////////////////////////////////////////////////////////////////
//-------------------------
// PHANToM�̔z�u
void Phantom::setPhantomConfiguration(char* view)
{
	if (!strcmp(view, "Normal")) { //strcmp�͓���������̏ꍇ0��Ԃ�
								   // Normal
		p2dMatrix[0] = 1; p2dMatrix[1] = 0; p2dMatrix[2] = 0;
		p2dMatrix[3] = 0; p2dMatrix[4] = 1; p2dMatrix[5] = 0;
		p2dMatrix[6] = 0; p2dMatrix[7] = 0; p2dMatrix[8] = 1;
	}
	else if (!strcmp(view, "Top")) {
		// from top
		p2dMatrix[0] = 1; p2dMatrix[1] = 0; p2dMatrix[2] = 0;
		p2dMatrix[3] = 0; p2dMatrix[4] = 0; p2dMatrix[5] = -1;
		p2dMatrix[6] = 0; p2dMatrix[7] = 1; p2dMatrix[8] = 0;
	}
	else if (!strcmp(view, "Right")) {
		// from right
		p2dMatrix[0] = 0; p2dMatrix[1] = 0; p2dMatrix[2] = -1;
		p2dMatrix[3] = 0; p2dMatrix[4] = 1; p2dMatrix[5] = 0;
		p2dMatrix[6] = 1; p2dMatrix[7] = 0; p2dMatrix[8] = 0;
	}
	else if (!strcmp(view, "RightTop")) {
		// from right and top
		p2dMatrix[0] = 0; p2dMatrix[1] = 0; p2dMatrix[2] = -1;
		p2dMatrix[3] = -1; p2dMatrix[4] = 0; p2dMatrix[5] = 0;
		p2dMatrix[6] = 0; p2dMatrix[7] = 1; p2dMatrix[8] = 0;
	}
	else if (!strcmp(view, "Left")) {
		// from left
		p2dMatrix[0] = 0; p2dMatrix[1] = 0; p2dMatrix[2] = 1;
		p2dMatrix[3] = 0; p2dMatrix[4] = 1; p2dMatrix[5] = 0;
		p2dMatrix[6] = -1; p2dMatrix[7] = 0; p2dMatrix[8] = 0;
	}
	else if (!strcmp(view, "LeftTop")) {
		// from left and top
		p2dMatrix[0] = 0; p2dMatrix[1] = 0; p2dMatrix[2] = 1;
		p2dMatrix[3] = 1; p2dMatrix[4] = 0; p2dMatrix[5] = 0;
		p2dMatrix[6] = 0; p2dMatrix[7] = 1; p2dMatrix[8] = 0;
	}
	trans(p2dMatrix, d2pMatrix);

}

//-------------------------
// ���������ǂݍ���
void Phantom::loadExpSequence(char *_fname)
{
	FILE *fpExp;
	char fname[256];
	//sprintf(fname, "%sPhantom%d.csv", _fname, number);
	sprintf(fname, "%sPhantom.csv", _fname);
	printf("%s\n", fname);//check

	fpExp = fopen(fname, "r");
	if (fpExp == NULL) {
		//printf("file open error!!\n");
		exit(0);
	}

	//Counting the Number of Trials 
	int num = 0;
	char str[1500];
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		num++;
		//printf("num = %d\n",num);
	}
	nTrials = num;
	printf("nTrials = %d\n", nTrials);
	fclose(fpExp);

	//Reading of the Experimental Conditions 
	fpExp = fopen(fname, "r");
	coCond = new int[nTrials];
	//ffCond      = new double[nTrials];
	vmCond = new double[nTrials];
	targetAngle = new double[nTrials];
	targetDistance = new double[nTrials];
	targetForce = new double[nTrials];
	targetColor = new int[nTrials];
	dErrorCramp = new double[nTrials]; // DDD:�G���[�N�����v�W��
	ConnectionType = new int[nTrials]; // connection type

									   // ffCond�̏�����
									   //�񎟌��z��̑傫�����m��
	ffCond = new double*[nTrials];
	for (int i = 0; i<nTrials; i++) {
		ffCond[i] = new double[NUMFFCOND];
		for (int j = 0; j<NUMFFCOND; j++) {
			ffCond[i][j] = 0.0;//0�ŏ�����
							   //printf("ffCond[%d][%d] = %f\n", i,j,ffCond[i][j]);
		}
	}

	num = 1;
	int tr;
	fgets(str, sizeof(str), fpExp);//�P�s�ڂ̓w�b�_�[�Ȃ̂Ŗ�������
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		if (num<nTrials) {
			// DDD:�G���[�N�����v�W����ǉ�
			sscanf(str, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf,%d",
				&tr, &coCond[num],
				&ffCond[num][0], &ffCond[num][1], &ffCond[num][2], &ffCond[num][3],
				&ffCond[num][4], &ffCond[num][5], &ffCond[num][6], &ffCond[num][7],
				&vmCond[num], &targetAngle[num], &targetDistance[num], &targetForce[num], &targetColor[num],
				&dErrorCramp[num],
				&ConnectionType[num]);
			printf("co[%d] = %d;  vm[%d] = %3.1lf; target[%d] = %3.1lf\n; ConnectionType[%d] = %d;",
				num, coCond[num], num, vmCond[num], num, targetAngle[num], num, ConnectionType[num]);
			num++;
		}
	}
	//0�Ԗڂɂ́A1�ԖڂƓ����������Z�b�g
	coCond[0] = coCond[1];
	//ffCond[0] = ffCond[1];
	vmCond[0] = vmCond[1];
	targetAngle[0] = targetAngle[1];
	targetDistance[0] = targetDistance[1];
	targetForce[0] = targetForce[1];
	targetColor[0] = targetColor[1];
	dErrorCramp[0] = dErrorCramp[1]; // DDD:�G���[�N�����v�W��
	for (int i = 0; i<8; i++) {	// DDD:vc8�Ή��ii��int��ǉ��j
		ffCond[0][i] = ffCond[1][i];
	}

	p2dScaleForForce = (targetDistance[0] * p2dScale) / targetForce[0];

	fclose(fpExp);
}

//-------------------------
// data�̏�����
void Phantom::initData(int _fs, int _recordTime, int _maxSample, int _numData)
{
	fs = _fs;
	recordTime = _recordTime;
	maxSample = _maxSample;
	numData = _numData;

	//*** data ***
	//�񎟌��z��̑傫�����m��
	data = new double*[maxSample];
	for (int i = 0; i<maxSample; i++) {
		data[i] = new double[numData];
	}
	//0�ŏ�����
	for (int i = 0; i<maxSample; i++) {	// DDD:vc8�Ή��ii��int��ǉ��j
		for (int j = 0; j<numData; j++) {
			data[i][j] = 0.0;
			//printf("data[%d][%d] = %f\n", i,j,data[i][j]);
		}
	}

	//*** time ***
	time = new int[maxSample];
	for (int i = 0; i<maxSample; i++) {	// DDD:vc8�Ή��ii��int��ǉ��j
		time[i] = 0.0;
	}
}

//-------------------------
// data�̏���
void Phantom::deleteData()
{
	int i;
	for (i = 0; i<maxSample; i++) {
		delete[] data[i];
	}
	delete[] data;
	delete[] time;

	delete[] coCond;
	//delete[] ffCond;
	delete[] vmCond;
	delete[] targetAngle;
	delete[] targetDistance;
	delete[] targetForce;
	delete[] targetColor;
	delete[] dErrorCramp; // DDD:�G���[�N�����v�W��

	for (i = 0; i<nTrials; i++) {
		delete[] ffCond[i];
	}
	delete[] ffCond;

	/* delete �� delete []�̈Ⴂ
	delete �� new �Ŋm�ۂ������̂��폜���邽�߂̂��̂ŁA
	delete [] �� new [] �i�z��̊��蓖�āj�Ŋm�ۂ������̂�
	�폜����ׂ̂��̂ł��B������A�z����폜���鎞�� delete [] ��
	�g��Ȃ��Ă͂Ȃ�܂���B�z��� delete �������Ă��ŏ��̗v�f
	�����폜���ꂸ�i�f�X�g���N�^���Ă΂ꂸ�j�A���\�[�X���[�N��
	�N�����Ă��܂��܂��B http://ray.sakura.ne.jp/tips/delete.html
	*/
}

//-------------------------
// data�̃Z�b�g
void Phantom::setData(int sample, int curTime, double instRate, double TrackingTarget[2])
{
	time[sample] = curTime;
	data[sample][0] = instRate;
	data[sample][1] = pPos[0];
	data[sample][2] = -pPos[2];
	data[sample][3] = pPos[1];
	data[sample][4] = pVel[0];
	data[sample][5] = -pVel[2];
	data[sample][6] = pVel[1];
	data[sample][7] = pForce[0];
	data[sample][8] = -pForce[2];
	data[sample][9] = pForce[1];

	/*
	//Gimbal
	data[sample][10] = pTrans[0];
	data[sample][11] = pTrans[1];
	data[sample][12] = pTrans[2];
	data[sample][13] = pTrans[4];
	data[sample][14] = pTrans[5];
	data[sample][15] = pTrans[6];
	data[sample][16] = pTrans[8];
	data[sample][17] = pTrans[9];
	data[sample][18] = pTrans[10];
	*/

	// Target
	data[sample][10] = TrackingTarget[0];
	data[sample][11] = TrackingTarget[1];
}

//-------------------------
// data�̕ۑ�
void Phantom::saveData(int trial, char* fname)
{
	char filename[500];
	sprintf(filename, "%sPhantom%d.csv", fname, number);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't open %s\n", filename);
		exit(0);


		//�f�B���N�g���쐬�R�}���h
		//_mkdir("data\\test");// <--direct.h���K�v
	}

	//printf("data[%d][%d] = %f\n", i,j,data[i][j]);


	// �������ݎ��s
	//�g���C�A�����̕ϐ�
	// DDD:�G���[�N�����v�W���ǉ�
	fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "trial", "coCond Force(0) Move(1)",
		"ffCond0 [N/(mm/s)]", "ffCond1 [N/(mm/s)]", "ffCond2 [N/(mm/s)]", "ffCond3 [N/(mm/s)]",
		"ffCond4 [N/(mm/s)]", "ffCond5 [N/(mm/s)]", "ffCond6 [N/(mm/s)]", "ffCond7 [N/(mm/s)]",
		"vmCond [deg]", "targetAngle[deg]", "targetDistance[mm]", "targetForce[N]", "targetColor", "errorCramp");
	fprintf(fp, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf\n", trial, coCond[trial],
		ffCond[trial][0], ffCond[trial][1], ffCond[trial][2], ffCond[trial][3],
		ffCond[trial][4], ffCond[trial][5], ffCond[trial][6], ffCond[trial][7],
		vmCond[trial], targetAngle[trial], targetDistance[trial], targetForce[trial], targetColor[trial],
		dErrorCramp[trial]);
	//���������@��ʓI�Ȑݒ�
	fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "fs[Hz]", "recordTime[ms]", "maxSample", "ks[N/mm]", "kb[N/(mm/s)]", "kConnection[N/mm]", "dConnection[N/(mm/s)]", "VisNoisePosSTD", "VisNoiseVelSTD", "ConnectionType");
	fprintf(fp, "%d,%d,%d,%lf,%lf,%lf,%lf,%2.1lf,%2.1lf,%d\n", fs, recordTime, maxSample, ks, kb, kConnection, dConnection, VisualNoisePosSTD, VisualNoiseVelSTD, ConnectionType[trial]);
	//���������@�X�P�[���֘A
	fprintf(fp, "%s,%s,%s, , ,%s\n", "targetRadius[mm]", "cursorRadius[mm]", "pOffset[mm]", "dOffset");
	fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", targetRadius, cursorRadius, pOffset[0], pOffset[1], pOffset[2], dOffset[0], dOffset[1], dOffset[2]);
	fprintf(fp, "%s,%s,%s\n", "p2dScale", "p2dScaleForForce", "p2dMatrix");
	fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", p2dScale, p2dScaleForForce, p2dMatrix[0], p2dMatrix[1], p2dMatrix[2], p2dMatrix[3], p2dMatrix[4], p2dMatrix[5], p2dMatrix[6], p2dMatrix[7], p2dMatrix[8]);
	//�󔒂R�s
	fprintf(fp, " , , ,\n");
	fprintf(fp, " , , ,\n");
	fprintf(fp, " , , ,\n");
	//�����f�[�^
	fprintf(fp, "%s,%s,%s,%s,%s,", "time[ms]", "instRate[Hz]", "posX[mm]", "posY[mm]", "posZ[mm]");
	fprintf(fp, "%s,%s,%s,", "velX[mm]", "velY[mm]", "velZ[mm]");
	fprintf(fp, "%s,%s,%s,", "forceX[N]", "forceY[N]", "forceZ[N]");
	//fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s," , "r11","r21","r31","r12","r22","r32","r13","r23","r33");
	fprintf(fp, "%s,%s\n", "TargetX", "TargetY");

	//���n��f�[�^�ۑ�
	for (int i = 0; i<maxSample; i++) {
		fprintf(fp, "%d,", time[i]);
		for (int j = 0; j<numData; j++) {
			if (j == (numData - 1))	fprintf(fp, "%f\n", data[i][j]);
			else				fprintf(fp, "%f,", data[i][j]);
		}
	}

	fclose(fp);

}

//***********************************
//
// �͂̌v�Z
//
//***********************************
//-------------------------
// �S���͂̌v�Z
void Phantom::calcConstraintForce(const int trial,
	const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime)
{
	f[0] = 0.0; f[1] = 0.0; f[2] = 0.0;

	// �t�@���g�����W�n�ɕϊ�
	HDdouble pConstraintForce[3] = { 0,0,0 }, dConstraintForce[3] = { 0,0,0 };
	dConstraintForce[2] = -ks * dPos[2];
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//�t�@���g�����W�n�ŗ͂��Z�b�g
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];

	/*

	//�v���g�[�ɒB����O�́A�S�����������߂ɁB
	double largeKb = 2*kb;//max3�{�܂�
	//****************************
	// ���_�S��
	//****************************
	if(coCond[trial]==0){
	HDdouble dConstraintForce[3] = {0,0,0};
	// stiffness
	// double ks = 0.5;//[N/mm]
	dConstraintForce[0] += - ks * dPos[0];
	dConstraintForce[1] += - ks * dPos[1];
	dConstraintForce[2] += - ks * dPos[2];
	// Damping
	//double kb = 0.0005;//[N/(mm/s)] 0.0005���Ə����S����������B
	//double kb = 0.0001;//[N/(mm/s)] 0.0001���ƂقƂ�Ǌ������Ȃ��B
	if (curTime<plateauTime){
	dConstraintForce[0] += - largeKb * dVel[0];
	dConstraintForce[1] += - largeKb * dVel[1];
	dConstraintForce[2] += - largeKb * dVel[2];
	//printf("pre\n");
	}
	else{
	dConstraintForce[0] += - kb * dVel[0];
	dConstraintForce[1] += - kb * dVel[1];
	dConstraintForce[2] += - kb * dVel[2];
	}

	// �t�@���g�����W�n�ɕϊ�
	HDdouble pConstraintForce[3] = {0,0,0};
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//�t�@���g�����W�n�ŗ͂��Z�b�g
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
	}
	//****************************
	// ���ʍS��(display���W�n��z=0����)
	//****************************
	else if (coCond[trial]==1){
	HDdouble dConstraintForce[3] = {0,0,0};
	//if (dPos[2]<0){
	//stiffness
	//double ks = 0.5;//[N/mm]
	dConstraintForce[2] += - ks * dPos[2];
	// Damping
	//double kb = 0.0005;//[N/(mm/s)] 0.0005���Ə����S����������B
	//double kb = 0.0001;//[N/(mm/s)] 0.0001���ƂقƂ�Ǌ������Ȃ��B
	if (curTime<plateauTime){
	dConstraintForce[0] += - largeKb * dVel[0];
	dConstraintForce[1] += - largeKb * dVel[1];
	dConstraintForce[2] += - largeKb * dVel[2];
	}
	else{
	dConstraintForce[0] += - kb * dVel[0];
	dConstraintForce[1] += - kb * dVel[1];
	dConstraintForce[2] += - kb * dVel[2];
	}
	//}
	// �t�@���g�����W�n�ɕϊ�
	HDdouble pConstraintForce[3] = {0,0,0};
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//�t�@���g�����W�n�ŗ͂��Z�b�g
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
	}
	*/
}

//-------------------------
// BackHomeForce�̌v�Z 2009/08/04
void Phantom::calcBackHomeForce(const int trial,
	const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome)
{
	f[0] = 0.0; f[1] = 0.0; f[2] = 0.0;
	//�v���g�[�ɒB����O�́A�S�����������߂ɁB
	double largeKb = 2 * kb;//max3�{�܂�
							//****************************
							// �񎟌����ʓ��ł̌��_�S����p����backHome����
							//****************************
	HDdouble dConstraintForce[3] = { 0,0,0 };
	// stiffness
	if (dPos[0]>0) dConstraintForce[0] += -0.5*std::log(1 + abs(dPos[0]));
	else           dConstraintForce[0] += +0.5*std::log(1 + abs(dPos[0]));
	if (dPos[1]>0) dConstraintForce[1] += -0.5*std::log(1 + abs(dPos[1]));
	else           dConstraintForce[1] += +0.5*std::log(1 + abs(dPos[1]));

	if (0) {
		double ksBackHome = 0.05;//[N/mm]//  <------------------
		dConstraintForce[0] += -ksBackHome * dPos[0];
		dConstraintForce[1] += -ksBackHome * dPos[1];
		//dConstraintForce[2] += - ksBackHome * dPos[2];
		//display���W�n�ɂ�����z���͂��łɕ��ʍS���Ōv�Z�ς݂Ȃ̂ŁA�����ł͍폜�B
	}

	// Damping
	double kbBackHome = (60 / 1000);//    <------------------
	dConstraintForce[0] += -kbBackHome * dVel[0];
	dConstraintForce[1] += -kbBackHome * dVel[1];
	//dConstraintForce[2] += - (60/1000) * dVel[2];

	//�͂����X�ɑ傫������
	int pTime = 1000;//                 <------------------
	double rate;
	rate = (double)cumulativeTimeForBackHome / (double)pTime;
	if (cumulativeTimeForBackHome < pTime) {
		scale(rate, dConstraintForce, dConstraintForce);
	}

	// �t�@���g�����W�n�ɕϊ�
	HDdouble pConstraintForce[3] = { 0,0,0 };
	mul(d2pMatrix, dConstraintForce, pConstraintForce);

	//�t�@���g�����W�n�ŗ͂��Z�b�g
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
}

//-------------------------------------------
// �t�H�[�X�t�B�[���h�s����擾
void Phantom::getForceFieldMatrix(int trial, double R[18])
{
	R[0] = ffCond[trial][0]; R[1] = ffCond[trial][1]; R[2] = 0; R[3] = ffCond[trial][4]; R[4] = ffCond[trial][5]; R[5] = 0;
	R[6] = ffCond[trial][2]; R[7] = ffCond[trial][3]; R[8] = 0; R[9] = ffCond[trial][6]; R[10] = ffCond[trial][7]; R[11] = 0;
	R[12] = 0;				  R[13] = 0;				R[14] = 0; R[15] = 0;                R[16] = 0; R[17] = 0;
}

//-------------------------------------------
// �t�H�[�X�t�B�[���h���v�Z
void Phantom::calcForceField(int trial, double gVel[6], double pEffectForce[3])
{
	// �͏�s��̃Z�b�g
	double R[18];
	getForceFieldMatrix(trial, R);

	// �͏���v�Z(�f�B�X�v���C���W�n�\��)
	double dEffectForce[3] = { 0,0,0 };
	mul36(R, gVel, dEffectForce);

	// �t�@���g�����W�n�ɕϊ�
	mul(d2pMatrix, dEffectForce, pEffectForce);
	//ffCond[trial]���v���X�Ȃ�A�����v�܂��
	//ffCond[trial]���}�C�i�X�Ȃ�A���v�܂��
	//ffCond[trial]���[���Ȃ�ANULL�g���C�A��
	//ffCond[trial] is around 0.005�`0.01[N/(mm/s)]
}

//-------------------------------------------
// DDD:�G���[�N�����v���v�Z
void Phantom::calcErrorCramp(int trial, double rate, double pErrorCramp[3])
{
	double T[2] = { dTargetPos[0], dTargetPos[1] }; // �^�[�Q�b�g�ʒu
	double C[2] = { dCursorPos[0], dCursorPos[1] }; // �J�[�\���ʒu
	double H[2]; // �i�J�n�ʒu-�^�[�Q�b�g�ʒu�j�����Ԓ�����ɃJ�[�\���ʒu���痎�Ƃ��������̑�
	double dErrorCrampForce[3] = { 0,0,0 };

	double S[2] = { -T[1], T[0] }; // �^�[�Q�b�g�Ɛ����ȃx�N�g��
	double V[2] = { dCursorVel[0], dCursorVel[1] }; //���x�x�N�g��
	double Vs[2] = { 0,0 };//���x�x�N�g���̃^�[�Q�b�g�����ɑ΂��Đ����Ȑ���

						   // �����̑�����уG���[�N�����v�͂̌v�Z
	double tmp = T[0] * T[0] + T[1] * T[1];

	if ((tmp != 0.0) && (dErrorCramp[trial] != 0.0))
	{
		//�e���W���ɂ���
		double k = (C[0] * T[0] + C[1] * T[1]) / tmp;
		H[0] = k * T[0];
		H[1] = k * T[1];

		//h�̓^�[�Q�b�g�����֐i�񂾋���
		double h = 0;
		double kErrorCramp = 0;
		h = (C[0] * T[0] + C[1] * T[1]) / std::sqrt(T[0] * T[0] + T[1] * T[1]);
		if ((h<20) && (h>0)) {
			double a;
			a = dErrorCramp[trial] / std::sqrt(20.0);
			kErrorCramp = a*std::sqrt(h);
		}
		else if (h <= 0) {
			kErrorCramp = 0;
		}
		else {
			kErrorCramp = dErrorCramp[trial];
		}

		//double r;
		//r = std::sqrt((H[0] - C[0])*(H[0] - C[0]) + ( H[1] - C[1] )*( H[1] - C[1] ));
		//if (r>2){
		dErrorCrampForce[0] = (H[0] - C[0]) * rate * kErrorCramp;
		dErrorCrampForce[1] = (H[1] - C[1]) * rate * kErrorCramp;
		dErrorCrampForce[2] = 0.0;
		//}

		//�S���W���ɂ���
		double kbCramp = 25.0 / 1000.0; //.0

		k = (V[0] * S[0] + V[1] * S[1]) / (S[0] * S[0] + S[1] * S[1]);
		Vs[0] = k * S[0];
		Vs[1] = k * S[1];

		dErrorCrampForce[0] += (-kbCramp * Vs[0]);
		dErrorCrampForce[1] += (-kbCramp * Vs[1]);
		dErrorCrampForce[2] += 0.0;
		//���̎��_��dErrorCrampForce�́AVM�������������W�n�\���ł���

		//VM���W�n����display���W�n�ɂ��ǂ�
		double R[9];
		double rad = -vmCond[trial] * (kPI / 180.0);//�����Ń}�C�i�X���������display���W�n�ɂ��ǂ�
		R[0] = std::cos(rad); R[1] = -std::sin(rad); R[2] = 0;
		R[3] = std::sin(rad); R[4] = std::cos(rad); R[5] = 0;
		R[6] = 0;        R[7] = 0;        R[8] = 0;
		mul(R, dErrorCrampForce, dErrorCrampForce);
	}

	// �t�@���g�����W�n�ɕϊ�
	mul(d2pMatrix, dErrorCrampForce, pErrorCramp);
}

//-------------------------------------------
// �^�[�Q�b�g��ڐG�\�ɂ���
void Phantom::calcHapticTarget(int trial, double pHapticTarget[3])
{
	double dHapticTarget[3] = { 0,0,0 };

	//�^�[�Q�b�g�ڐG�@�\���g�������ꍇ�́Aif (1)�Ƃ���Ύg����B
	if (0) {
		//�S���W���ɂ���
		double kbHaptic = 10.0 / 1000.0;
		if (inTarget) {
			printf("Now inTarget!!! \n");
			dHapticTarget[0] += (-kbHaptic * dVel[0]);
			dHapticTarget[1] += (-kbHaptic * dVel[1]);
		}
	}

	// �t�@���g�����W�n�ɕϊ�
	mul(d2pMatrix, dHapticTarget, pHapticTarget);
}

//***********************************
//
// �ʒu�E���x�̌v�Z
//
//***********************************
//-------------------------------------------
// Phantom���W�n����f�B�X�v���C���W�n�ɕϊ�
// �ʒu�ɂ���
void Phantom::calcDPos(double pPos[3], double dPos[3])
{
	//(����) pPos���ύX�����I�I
	//���_�̕��i�ʒu�̕ϊ�
	pPos[0] -= pOffset[0];
	pPos[1] -= pOffset[1];
	pPos[2] -= pOffset[2];
	//��]�ϊ�
	mul(p2dMatrix, pPos, dPos);
}
// ���x�ɂ���
void Phantom::calcDVel(double pVel[3], double dVel[3])
{
	//��]�ϊ�
	mul(p2dMatrix, pVel, dVel);
}

//-------------------------------------------
// //�f�B�X�v���C���W�n�̃X�P�[���ɕϊ�
void Phantom::calcDisplayScaleParameter(int trial)
{
	// �f�B�X�v���C��̃^�[�Q�b�g�̑傫���̒��Ɉ����ԓ���΁A���s�J�n�Ƃ���I
	// Bad translation ^^ If placed in a certain period of time in the size of the target 
	// the display, as a start try!
	//(1)calculate the evaluation criteria 
	calcDTargetRadius();
	//(2) calculating an evaluation value
	calcDCursorPosVel(trial);
	//(3) Calculating a target position on the display 
	calcDTargetPos(trial);
}
//(1)
void Phantom::calcDTargetRadius()
{
	//assinged to the instance variable 
	dTargetRadius = targetRadius * p2dScale;
}
//(2)
void Phantom::calcDCursorPosVel(int trial)
{
	//*********************
	// dCursorPos�v�Z
	//*********************
	double cursorPos[3] = { 0,0,0 };
	if (coCond[trial] == 0) {
		//force�ۑ�ł́A�͂��]���l
		double dForce[3] = { 0,0,0 };
		double p_force[3] = { -pForce[0], -pForce[1],-pForce[2] };
		mul(p2dMatrix, p_force, dForce);
		scale(p2dScaleForForce, dForce, cursorPos);//�f�B�X�v���C���W�n�̃X�P�[���ɕϊ�
	}
	else if (coCond[trial] == 1) {
		//Move�ۑ�ł́A�ʒu���]���l
		scale(p2dScale, dPos, cursorPos);//�f�B�X�v���C���W�n�̃X�P�[���ɕϊ�
	}

	//2009/07/23 Hirashima
	//�����F������VM��]�������邱�ƂŁAdCursorPos��VM��]��̒l�������Ă���B
	// dCursorPos��p���āAcalcErrorCramp, detectInTarget���s���Ă���̂ŁA
	// ���̂Q�̊֐��́AVM���l�������v�Z�ɂȂ����B
	//Visuomotor rotation
	//state.vmCond_���v���X�Ȃ甽���v���
	double R[9];
	double rad = vmCond[trial] * (kPI / 180.0);
	R[0] = std::cos(rad); R[1] = -std::sin(rad); R[2] = 0;
	R[3] = std::sin(rad); R[4] = std::cos(rad); R[5] = 0;
	R[6] = 0;			  R[7] = 0;				R[8] = 0;
	mul(R, cursorPos, cursorPos);

	//Assigned to the Instance Variable 
	dCursorPos[0] = cursorPos[0];
	dCursorPos[1] = cursorPos[1];
	dCursorPos[2] = cursorPos[2];

	//*********************
	// dCursorVel�v�Z
	//*********************
	double cursorVel[3] = { 0,0,0 };
	if (coCond[trial] == 0) {
	}
	else if (coCond[trial] == 1) {//Move�ۑ�
		scale(p2dScale, dVel, cursorVel);//�f�B�X�v���C���W�n�̃X�P�[���ɕϊ�
	}
	//VM��]��������
	mul(R, cursorVel, cursorVel);
	//�C���X�^���X�ϐ��ɑ��
	dCursorVel[0] = cursorVel[0];
	dCursorVel[1] = cursorVel[1];
	dCursorVel[2] = cursorVel[2];
}

//(3)
void Phantom::calcDTargetPos(int trial)
{
	double targetPos[3] = { 0,0,0 };
	// ������ݒ�
	targetPos[0] = std::cos(targetAngle[trial] * (kPI / 180.0));
	targetPos[1] = std::sin(targetAngle[trial] * (kPI / 180.0));
	targetPos[2] = 0.0;
	// �傫����ݒ�
	if (coCond[trial] == 0) {
		//force�ۑ�
		scale(targetForce[trial], targetPos, targetPos);//Set the size of the force 
		scale(p2dScaleForForce, targetPos, targetPos);//display�p�ɃT�C�Y�ύX
	}
	else if (coCond[trial] == 1) {
		//Move�ۑ�
		scale(targetDistance[trial], targetPos, targetPos);//�^���̋������Z�b�g
		scale(p2dScale, targetPos, targetPos);//display�p�ɃT�C�Y�ύX
	}
	//Assinged to the instance variable 
	dTargetPos[0] = targetPos[0];
	dTargetPos[1] = targetPos[1];
	dTargetPos[2] = targetPos[2];
}

//--------------------------------------------
// �^�[�Q�b�g�ւ̓��B�𔻒�
void Phantom::detectInTarget(int curTime)
{
	double rx = dTargetPos[0] - dCursorPos[0];
	double ry = dTargetPos[1] - dCursorPos[1];
	double r = std::sqrt((rx*rx) + (ry*ry));
	//����
	if (r < dTargetRadius) {

		//���@�\���g�������ꍇ�́Aif (1)�Ƃ���Ύg����B
		if (0) {
			//---------------------------------
			if (!inTarget && (curTime > soundTime + soundRestTime)) {
				if (number == 0) PlaySound("sound/coin.wav", NULL, SND_FILENAME | SND_ASYNC);
				else if (number == 1) PlaySound("sound/bokan.wav", NULL, SND_FILENAME | SND_ASYNC);
				soundTime = curTime;
			}
			//---------------------------------
		}



		inTarget = true;
	}
	else { inTarget = false; }
}

//--------------------------------------------
// �����ʒu�ɓ����Ă��邱�Ƃ𔻒�
void Phantom::detectInInitialPos()
{
	double rx = dCursorPos[0];
	double ry = dCursorPos[1];
	double r = std::sqrt((rx*rx) + (ry*ry));
	//����
	if (r < (0.5*dTargetRadius)) { inInitialPos = true; }
	else { inInitialPos = false; }
}

//----------------------------------
// �s�[�N���x����
void Phantom::detectPeakVel(int sample)
{
	double curVel = norm(pVel);
	//���菈���ɓ���
	if (!inDetectPeakVel) {
		inDetectPeakVel = true;
		peakVel = curVel;
	}
	else {
		//���݂�norm���傫��������A�X�V
		if (peakVel < curVel) {
			peakVel = curVel;
		}
	}
	//---------------------------------------------
	// �s���̓��삪�I��������炢�ŁA
	// �s�[�N���x�̔�����s���A���菈�����I������
	//A = 100mm, D = 400msec, -> Vpeak = 1.88 * A/D = 470mm/sec (minimum jerk model)
	int	halfMaxSample = std::floor((double)(maxSample / 2));	// DDD:vc8�Ή��ifloor�̈����ɂĖ����I�L���X�g�j
	if (sample == halfMaxSample) {
		if (peakVel > 515)		velRank = FAST;//+-10%
		else if (peakVel < 425)	velRank = SLOW;
		else					velRank = GOOD;

		//printf("sample = %d\n", sample);
		//printf("phantom[%d] velRank = %d, peakVel = %f\n", number, velRank, peakVel);

		//���菈�����I�����ApeakVel���[���ɖ߂�
		inDetectPeakVel = false;
		peakVel = 0;
	}
}


