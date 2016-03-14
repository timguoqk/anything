// Phantom.cpp: Phantom クラスのインプリメンテーション
//
//////////////////////////////////////////////////////////////////////

#include "Phantom.h"
#include "helper.h"

#include <cmath>	// for : log, sqrt, sin, cos


//////////////////////////////////////////////////////////////////////
// コンストラクタ
//////////////////////////////////////////////////////////////////////
Phantom::Phantom(int _number, HDstring _name, char* _view,
	double _pOffset[3], double _dOffset[3],
	double _p2dScale, double _ks, double _kb)
{
	number = _number;
	name = _name;
	printf("PHANToM[%d] = %s\n", number, name);

	// 被験者に対するファントムの姿勢をセット
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
	//dOffset[mm]: Cursor on the display, when you draw a target, display from plus only dOffset。
	dOffset[0] = _dOffset[0];
	dOffset[1] = _dOffset[1];
	dOffset[2] = _dOffset[2];

	//実世界からdisplay世界へのスケール変換
	p2dScale = _p2dScale;
	//const double p2dScaleForForce = (targetDistance * p2dScale)/targetForce;
	//display上でのターゲットの距離が同じになるようにセット

	//-------------------------
	// 色
	memcpy(cursorCol, white, sizeof(white));

	//-------------------------
	// 実験条件 --> loadExpSequenceに移行済み
	//int    *coCond;//constraint condition
	//double *ffCond;//force field condition
	//double *vmCond;//visuomotor condition
	//double *targetAngle;

	//-------------------------
	// PLANAR CONSTRAINT STRENGTH
	ks = 0.8;//stiffness [N/mm]//desktopなら0.5～1.0 //premiumなら2.0もOK
	kb = 0.0001;
	//damping [N/(mm/s)] 0.0005だと少し粘性を感じる。
	//0.0001だとほとんどわからない
	// Connection strengths
	kConnection = 0.1;
	dConnection = 0.0001;



	//Force at phantom coordinate system(updated in the servo loop）
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

	soundTime = 0;//サウンドの不応期をつくるために必要
	soundRestTime = 1000;//サウンド不応期[ms]

						 //Gimbal変数初期化
						 //setZero(gAng);
	button = 0;
	for (int i = 0; i<16; i++) {
		pTrans[i] = 0.0;
	}

	fp = NULL; // DDD:一度も到達しなかった場合でも正常終了させるため
}
//////////////////////////////////////////////////////////////////////
// デストラクタ
//////////////////////////////////////////////////////////////////////
Phantom::~Phantom()
{
	deleteData();
}

//////////////////////////////////////////////////////////////////////
// クラスメソッド
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
// インスタンスメソッド
//////////////////////////////////////////////////////////////////////
//-------------------------
// PHANToMの配置
void Phantom::setPhantomConfiguration(char* view)
{
	if (!strcmp(view, "Normal")) { //strcmpは同じ文字列の場合0を返す
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
// 実験条件読み込み
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
	dErrorCramp = new double[nTrials]; // DDD:エラークランプ係数
	ConnectionType = new int[nTrials]; // connection type

									   // ffCondの初期化
									   //二次元配列の大きさを確保
	ffCond = new double*[nTrials];
	for (int i = 0; i<nTrials; i++) {
		ffCond[i] = new double[NUMFFCOND];
		for (int j = 0; j<NUMFFCOND; j++) {
			ffCond[i][j] = 0.0;//0で初期化
							   //printf("ffCond[%d][%d] = %f\n", i,j,ffCond[i][j]);
		}
	}

	num = 1;
	int tr;
	fgets(str, sizeof(str), fpExp);//１行目はヘッダーなので無視する
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		if (num<nTrials) {
			// DDD:エラークランプ係数を追加
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
	//0番目には、1番目と同じ条件をセット
	coCond[0] = coCond[1];
	//ffCond[0] = ffCond[1];
	vmCond[0] = vmCond[1];
	targetAngle[0] = targetAngle[1];
	targetDistance[0] = targetDistance[1];
	targetForce[0] = targetForce[1];
	targetColor[0] = targetColor[1];
	dErrorCramp[0] = dErrorCramp[1]; // DDD:エラークランプ係数
	for (int i = 0; i<8; i++) {	// DDD:vc8対応（iにintを追加）
		ffCond[0][i] = ffCond[1][i];
	}

	p2dScaleForForce = (targetDistance[0] * p2dScale) / targetForce[0];

	fclose(fpExp);
}

//-------------------------
// dataの初期化
void Phantom::initData(int _fs, int _recordTime, int _maxSample, int _numData)
{
	fs = _fs;
	recordTime = _recordTime;
	maxSample = _maxSample;
	numData = _numData;

	//*** data ***
	//二次元配列の大きさを確保
	data = new double*[maxSample];
	for (int i = 0; i<maxSample; i++) {
		data[i] = new double[numData];
	}
	//0で初期化
	for (int i = 0; i<maxSample; i++) {	// DDD:vc8対応（iにintを追加）
		for (int j = 0; j<numData; j++) {
			data[i][j] = 0.0;
			//printf("data[%d][%d] = %f\n", i,j,data[i][j]);
		}
	}

	//*** time ***
	time = new int[maxSample];
	for (int i = 0; i<maxSample; i++) {	// DDD:vc8対応（iにintを追加）
		time[i] = 0.0;
	}
}

//-------------------------
// dataの消去
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
	delete[] dErrorCramp; // DDD:エラークランプ係数

	for (i = 0; i<nTrials; i++) {
		delete[] ffCond[i];
	}
	delete[] ffCond;

	/* delete と delete []の違い
	delete は new で確保したものを削除するためのもので、
	delete [] は new [] （配列の割り当て）で確保したものを
	削除する為のものです。だから、配列を削除する時は delete [] を
	使わなくてはなりません。配列に delete をかけても最初の要素
	しか削除されず（デストラクタも呼ばれず）、リソースリークが
	起こってしまいます。 http://ray.sakura.ne.jp/tips/delete.html
	*/
}

//-------------------------
// dataのセット
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
// dataの保存
void Phantom::saveData(int trial, char* fname)
{
	char filename[500];
	sprintf(filename, "%sPhantom%d.csv", fname, number);
	fp = fopen(filename, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't open %s\n", filename);
		exit(0);


		//ディレクトリ作成コマンド
		//_mkdir("data\\test");// <--direct.hが必要
	}

	//printf("data[%d][%d] = %f\n", i,j,data[i][j]);


	// 書き込み実行
	//トライアル毎の変数
	// DDD:エラークランプ係数追加
	fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "trial", "coCond Force(0) Move(1)",
		"ffCond0 [N/(mm/s)]", "ffCond1 [N/(mm/s)]", "ffCond2 [N/(mm/s)]", "ffCond3 [N/(mm/s)]",
		"ffCond4 [N/(mm/s)]", "ffCond5 [N/(mm/s)]", "ffCond6 [N/(mm/s)]", "ffCond7 [N/(mm/s)]",
		"vmCond [deg]", "targetAngle[deg]", "targetDistance[mm]", "targetForce[N]", "targetColor", "errorCramp");
	fprintf(fp, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf\n", trial, coCond[trial],
		ffCond[trial][0], ffCond[trial][1], ffCond[trial][2], ffCond[trial][3],
		ffCond[trial][4], ffCond[trial][5], ffCond[trial][6], ffCond[trial][7],
		vmCond[trial], targetAngle[trial], targetDistance[trial], targetForce[trial], targetColor[trial],
		dErrorCramp[trial]);
	//実験条件　一般的な設定
	fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "fs[Hz]", "recordTime[ms]", "maxSample", "ks[N/mm]", "kb[N/(mm/s)]", "kConnection[N/mm]", "dConnection[N/(mm/s)]", "VisNoisePosSTD", "VisNoiseVelSTD", "ConnectionType");
	fprintf(fp, "%d,%d,%d,%lf,%lf,%lf,%lf,%2.1lf,%2.1lf,%d\n", fs, recordTime, maxSample, ks, kb, kConnection, dConnection, VisualNoisePosSTD, VisualNoiseVelSTD, ConnectionType[trial]);
	//実験条件　スケール関連
	fprintf(fp, "%s,%s,%s, , ,%s\n", "targetRadius[mm]", "cursorRadius[mm]", "pOffset[mm]", "dOffset");
	fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", targetRadius, cursorRadius, pOffset[0], pOffset[1], pOffset[2], dOffset[0], dOffset[1], dOffset[2]);
	fprintf(fp, "%s,%s,%s\n", "p2dScale", "p2dScaleForForce", "p2dMatrix");
	fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", p2dScale, p2dScaleForForce, p2dMatrix[0], p2dMatrix[1], p2dMatrix[2], p2dMatrix[3], p2dMatrix[4], p2dMatrix[5], p2dMatrix[6], p2dMatrix[7], p2dMatrix[8]);
	//空白３行
	fprintf(fp, " , , ,\n");
	fprintf(fp, " , , ,\n");
	fprintf(fp, " , , ,\n");
	//実験データ
	fprintf(fp, "%s,%s,%s,%s,%s,", "time[ms]", "instRate[Hz]", "posX[mm]", "posY[mm]", "posZ[mm]");
	fprintf(fp, "%s,%s,%s,", "velX[mm]", "velY[mm]", "velZ[mm]");
	fprintf(fp, "%s,%s,%s,", "forceX[N]", "forceY[N]", "forceZ[N]");
	//fprintf(fp, "%s,%s,%s,%s,%s,%s,%s,%s,%s," , "r11","r21","r31","r12","r22","r32","r13","r23","r33");
	fprintf(fp, "%s,%s\n", "TargetX", "TargetY");

	//時系列データ保存
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
// 力の計算
//
//***********************************
//-------------------------
// 拘束力の計算
void Phantom::calcConstraintForce(const int trial,
	const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime)
{
	f[0] = 0.0; f[1] = 0.0; f[2] = 0.0;

	// ファントム座標系に変換
	HDdouble pConstraintForce[3] = { 0,0,0 }, dConstraintForce[3] = { 0,0,0 };
	dConstraintForce[2] = -ks * dPos[2];
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//ファントム座標系で力をセット
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];

	/*

	//プラトーに達する前は、粘性を少し多めに。
	double largeKb = 2*kb;//max3倍まで
	//****************************
	// 原点拘束
	//****************************
	if(coCond[trial]==0){
	HDdouble dConstraintForce[3] = {0,0,0};
	// stiffness
	// double ks = 0.5;//[N/mm]
	dConstraintForce[0] += - ks * dPos[0];
	dConstraintForce[1] += - ks * dPos[1];
	dConstraintForce[2] += - ks * dPos[2];
	// Damping
	//double kb = 0.0005;//[N/(mm/s)] 0.0005だと少し粘性を感じる。
	//double kb = 0.0001;//[N/(mm/s)] 0.0001だとほとんど感じられない。
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

	// ファントム座標系に変換
	HDdouble pConstraintForce[3] = {0,0,0};
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//ファントム座標系で力をセット
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
	}
	//****************************
	// 平面拘束(display座標系のz=0平面)
	//****************************
	else if (coCond[trial]==1){
	HDdouble dConstraintForce[3] = {0,0,0};
	//if (dPos[2]<0){
	//stiffness
	//double ks = 0.5;//[N/mm]
	dConstraintForce[2] += - ks * dPos[2];
	// Damping
	//double kb = 0.0005;//[N/(mm/s)] 0.0005だと少し粘性を感じる。
	//double kb = 0.0001;//[N/(mm/s)] 0.0001だとほとんど感じられない。
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
	// ファントム座標系に変換
	HDdouble pConstraintForce[3] = {0,0,0};
	mul(d2pMatrix, dConstraintForce, pConstraintForce);
	//ファントム座標系で力をセット
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
	}
	*/
}

//-------------------------
// BackHomeForceの計算 2009/08/04
void Phantom::calcBackHomeForce(const int trial,
	const double dPos[3], const double dVel[3], double f[3], int curTime, int plateauTime, int cumulativeTimeForBackHome)
{
	f[0] = 0.0; f[1] = 0.0; f[2] = 0.0;
	//プラトーに達する前は、粘性を少し多めに。
	double largeKb = 2 * kb;//max3倍まで
							//****************************
							// 二次元平面内での原点拘束を用いてbackHomeする
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
		//display座標系におけるz軸はすでに平面拘束で計算済みなので、ここでは削除。
	}

	// Damping
	double kbBackHome = (60 / 1000);//    <------------------
	dConstraintForce[0] += -kbBackHome * dVel[0];
	dConstraintForce[1] += -kbBackHome * dVel[1];
	//dConstraintForce[2] += - (60/1000) * dVel[2];

	//力を徐々に大きくする
	int pTime = 1000;//                 <------------------
	double rate;
	rate = (double)cumulativeTimeForBackHome / (double)pTime;
	if (cumulativeTimeForBackHome < pTime) {
		scale(rate, dConstraintForce, dConstraintForce);
	}

	// ファントム座標系に変換
	HDdouble pConstraintForce[3] = { 0,0,0 };
	mul(d2pMatrix, dConstraintForce, pConstraintForce);

	//ファントム座標系で力をセット
	f[0] += pConstraintForce[0];
	f[1] += pConstraintForce[1];
	f[2] += pConstraintForce[2];
}

//-------------------------------------------
// フォースフィールド行列を取得
void Phantom::getForceFieldMatrix(int trial, double R[18])
{
	R[0] = ffCond[trial][0]; R[1] = ffCond[trial][1]; R[2] = 0; R[3] = ffCond[trial][4]; R[4] = ffCond[trial][5]; R[5] = 0;
	R[6] = ffCond[trial][2]; R[7] = ffCond[trial][3]; R[8] = 0; R[9] = ffCond[trial][6]; R[10] = ffCond[trial][7]; R[11] = 0;
	R[12] = 0;				  R[13] = 0;				R[14] = 0; R[15] = 0;                R[16] = 0; R[17] = 0;
}

//-------------------------------------------
// フォースフィールドを計算
void Phantom::calcForceField(int trial, double gVel[6], double pEffectForce[3])
{
	// 力場行列のセット
	double R[18];
	getForceFieldMatrix(trial, R);

	// 力場を計算(ディスプレイ座標系表現)
	double dEffectForce[3] = { 0,0,0 };
	mul36(R, gVel, dEffectForce);

	// ファントム座標系に変換
	mul(d2pMatrix, dEffectForce, pEffectForce);
	//ffCond[trial]がプラスなら、反時計まわり
	//ffCond[trial]がマイナスなら、時計まわり
	//ffCond[trial]がゼロなら、NULLトライアル
	//ffCond[trial] is around 0.005～0.01[N/(mm/s)]
}

//-------------------------------------------
// DDD:エラークランプを計算
void Phantom::calcErrorCramp(int trial, double rate, double pErrorCramp[3])
{
	double T[2] = { dTargetPos[0], dTargetPos[1] }; // ターゲット位置
	double C[2] = { dCursorPos[0], dCursorPos[1] }; // カーソル位置
	double H[2]; // （開始位置-ターゲット位置）を結ぶ直線上にカーソル位置から落とした垂線の足
	double dErrorCrampForce[3] = { 0,0,0 };

	double S[2] = { -T[1], T[0] }; // ターゲットと垂直なベクトル
	double V[2] = { dCursorVel[0], dCursorVel[1] }; //速度ベクトル
	double Vs[2] = { 0,0 };//速度ベクトルのターゲット方向に対して垂直な成分

						   // 垂線の足およびエラークランプ力の計算
	double tmp = T[0] * T[0] + T[1] * T[1];

	if ((tmp != 0.0) && (dErrorCramp[trial] != 0.0))
	{
		//弾性係数について
		double k = (C[0] * T[0] + C[1] * T[1]) / tmp;
		H[0] = k * T[0];
		H[1] = k * T[1];

		//hはターゲット方向へ進んだ距離
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

		//粘性係数について
		double kbCramp = 25.0 / 1000.0; //.0

		k = (V[0] * S[0] + V[1] * S[1]) / (S[0] * S[0] + S[1] * S[1]);
		Vs[0] = k * S[0];
		Vs[1] = k * S[1];

		dErrorCrampForce[0] += (-kbCramp * Vs[0]);
		dErrorCrampForce[1] += (-kbCramp * Vs[1]);
		dErrorCrampForce[2] += 0.0;
		//この時点のdErrorCrampForceは、VMがかかった座標系表現である

		//VM座標系からdisplay座標系にもどす
		double R[9];
		double rad = -vmCond[trial] * (kPI / 180.0);//ここでマイナスをかければdisplay座標系にもどる
		R[0] = std::cos(rad); R[1] = -std::sin(rad); R[2] = 0;
		R[3] = std::sin(rad); R[4] = std::cos(rad); R[5] = 0;
		R[6] = 0;        R[7] = 0;        R[8] = 0;
		mul(R, dErrorCrampForce, dErrorCrampForce);
	}

	// ファントム座標系に変換
	mul(d2pMatrix, dErrorCrampForce, pErrorCramp);
}

//-------------------------------------------
// ターゲットを接触可能にする
void Phantom::calcHapticTarget(int trial, double pHapticTarget[3])
{
	double dHapticTarget[3] = { 0,0,0 };

	//ターゲット接触機能を使いたい場合は、if (1)とすれば使える。
	if (0) {
		//粘性係数について
		double kbHaptic = 10.0 / 1000.0;
		if (inTarget) {
			printf("Now inTarget!!! \n");
			dHapticTarget[0] += (-kbHaptic * dVel[0]);
			dHapticTarget[1] += (-kbHaptic * dVel[1]);
		}
	}

	// ファントム座標系に変換
	mul(d2pMatrix, dHapticTarget, pHapticTarget);
}

//***********************************
//
// 位置・速度の計算
//
//***********************************
//-------------------------------------------
// Phantom座標系からディスプレイ座標系に変換
// 位置について
void Phantom::calcDPos(double pPos[3], double dPos[3])
{
	//(注意) pPosも変更される！！
	//原点の並進位置の変換
	pPos[0] -= pOffset[0];
	pPos[1] -= pOffset[1];
	pPos[2] -= pOffset[2];
	//回転変換
	mul(p2dMatrix, pPos, dPos);
}
// 速度について
void Phantom::calcDVel(double pVel[3], double dVel[3])
{
	//回転変換
	mul(p2dMatrix, pVel, dVel);
}

//-------------------------------------------
// //ディスプレイ座標系のスケールに変換
void Phantom::calcDisplayScaleParameter(int trial)
{
	// ディスプレイ上のターゲットの大きさの中に一定期間入れば、試行開始とする！
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
	// dCursorPos計算
	//*********************
	double cursorPos[3] = { 0,0,0 };
	if (coCond[trial] == 0) {
		//force課題では、力が評価値
		double dForce[3] = { 0,0,0 };
		double p_force[3] = { -pForce[0], -pForce[1],-pForce[2] };
		mul(p2dMatrix, p_force, dForce);
		scale(p2dScaleForForce, dForce, cursorPos);//ディスプレイ座標系のスケールに変換
	}
	else if (coCond[trial] == 1) {
		//Move課題では、位置が評価値
		scale(p2dScale, dPos, cursorPos);//ディスプレイ座標系のスケールに変換
	}

	//2009/07/23 Hirashima
	//メモ：ここでVM回転をかけることで、dCursorPosはVM回転後の値が入っている。
	// dCursorPosを用いて、calcErrorCramp, detectInTargetを行っているので、
	// この２つの関数は、VMを考慮した計算になった。
	//Visuomotor rotation
	//state.vmCond_がプラスなら反時計回り
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
	// dCursorVel計算
	//*********************
	double cursorVel[3] = { 0,0,0 };
	if (coCond[trial] == 0) {
	}
	else if (coCond[trial] == 1) {//Move課題
		scale(p2dScale, dVel, cursorVel);//ディスプレイ座標系のスケールに変換
	}
	//VM回転をかける
	mul(R, cursorVel, cursorVel);
	//インスタンス変数に代入
	dCursorVel[0] = cursorVel[0];
	dCursorVel[1] = cursorVel[1];
	dCursorVel[2] = cursorVel[2];
}

//(3)
void Phantom::calcDTargetPos(int trial)
{
	double targetPos[3] = { 0,0,0 };
	// 方向を設定
	targetPos[0] = std::cos(targetAngle[trial] * (kPI / 180.0));
	targetPos[1] = std::sin(targetAngle[trial] * (kPI / 180.0));
	targetPos[2] = 0.0;
	// 大きさを設定
	if (coCond[trial] == 0) {
		//force課題
		scale(targetForce[trial], targetPos, targetPos);//Set the size of the force 
		scale(p2dScaleForForce, targetPos, targetPos);//display用にサイズ変更
	}
	else if (coCond[trial] == 1) {
		//Move課題
		scale(targetDistance[trial], targetPos, targetPos);//運動の距離をセット
		scale(p2dScale, targetPos, targetPos);//display用にサイズ変更
	}
	//Assinged to the instance variable 
	dTargetPos[0] = targetPos[0];
	dTargetPos[1] = targetPos[1];
	dTargetPos[2] = targetPos[2];
}

//--------------------------------------------
// ターゲットへの到達を判定
void Phantom::detectInTarget(int curTime)
{
	double rx = dTargetPos[0] - dCursorPos[0];
	double ry = dTargetPos[1] - dCursorPos[1];
	double r = std::sqrt((rx*rx) + (ry*ry));
	//判定
	if (r < dTargetRadius) {

		//音機能を使いたい場合は、if (1)とすれば使える。
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
// 初期位置に入っていることを判定
void Phantom::detectInInitialPos()
{
	double rx = dCursorPos[0];
	double ry = dCursorPos[1];
	double r = std::sqrt((rx*rx) + (ry*ry));
	//判定
	if (r < (0.5*dTargetRadius)) { inInitialPos = true; }
	else { inInitialPos = false; }
}

//----------------------------------
// ピーク速度判定
void Phantom::detectPeakVel(int sample)
{
	double curVel = norm(pVel);
	//判定処理に入る
	if (!inDetectPeakVel) {
		inDetectPeakVel = true;
		peakVel = curVel;
	}
	else {
		//現在のnormが大きかったら、更新
		if (peakVel < curVel) {
			peakVel = curVel;
		}
	}
	//---------------------------------------------
	// 行きの動作が終わったくらいで、
	// ピーク速度の判定を行い、判定処理を終了する
	//A = 100mm, D = 400msec, -> Vpeak = 1.88 * A/D = 470mm/sec (minimum jerk model)
	int	halfMaxSample = std::floor((double)(maxSample / 2));	// DDD:vc8対応（floorの引数にて明示的キャスト）
	if (sample == halfMaxSample) {
		if (peakVel > 515)		velRank = FAST;//+-10%
		else if (peakVel < 425)	velRank = SLOW;
		else					velRank = GOOD;

		//printf("sample = %d\n", sample);
		//printf("phantom[%d] velRank = %d, peakVel = %f\n", number, velRank, peakVel);

		//判定処理を終了し、peakVelをゼロに戻す
		inDetectPeakVel = false;
		peakVel = 0;
	}
}


