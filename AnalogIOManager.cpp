// インクルード
#include <stdio.h>
#include "AnalogIOManager.h"	// CAnalogIOManagerクラスヘッダ

//#define USE_CAIO_LIB //このようにdefineすれば、contec board を利用するモードになることを確認. Hirashima 2014/03/20.
// Declare include header and linking library for using contec DA board depend on the preprocessor macro.
#ifdef USE_CAIO_LIB
#include "Caio.h"					// アナログ入出力ボード用 API ヘッダのインクルード
#pragma comment( lib, "Caio.lib" )	// アナログ入出力ボードライブラリのリンク
#endif


// 定数の定義

// ローカル関数定義

// アナログ入出力タイマー関数
#ifdef _WIN64
void CALLBACK TimerProc(UINT      uID,    // タイマーID
	UINT      uMsg,   // 予約
	DWORD_PTR dwUser, // ユーザー定義
	DWORD_PTR dw1,    // 予約
	DWORD_PTR dw2     // 予約
	);
#else
void CALLBACK TimerProc(UINT uID,     // タイマーID
	UINT uMsg,    // 予約
	DWORD dwUser, // ユーザー定義
	DWORD dw1,    // 予約
	DWORD dw2     // 予約
	);
#endif

// コンストラクタ
// 各種変数の初期化
CAnalogIOManager::CAnalogIOManager(void)
{
	currentTrial = 0;
	bInTrial = false;
	bInSave = false;
}

// デストラクタ */
// 各種ポインタ変数の終了化
CAnalogIOManager::~CAnalogIOManager(void)
{
}

// 初期化
bool CAnalogIOManager::init()
{

#ifdef USE_CAIO_LIB

	long ret = AioInit("AIO000", &id);

	// アナログ入出力ボードの初期化
	if (ret != 0)
	{
		printf("AioInitでエラーが発生しました %d\n", ret);
		return false;
	}

	// レンジの設定
	AioSetAiRangeAll(id, PM10); // //±10V

	return true;

#else

	return false;

#endif
}

// 終了化
bool CAnalogIOManager::destroy()
{

#ifdef USE_CAIO_LIB

	long ret = AioExit(id);

	// アナログ入出力ボードの終了化
	if (ret != 0)
	{
		printf("AioExitでエラーが発生しました %d\n", ret);
		return false;
	}

	// マルチメディアタイマーの終了化
	timeKillEvent(timerId);
	timeEndPeriod(1);

	return true;

#else

	return false;

#endif
}

// 入力データ配列の初期化
void CAnalogIOManager::initInputData(int _fs, int _recordTime, int _maxSample)
{
#ifdef USE_CAIO_LIB

	fs = _fs;
	recordTime = _recordTime;
	maxSample = _maxSample;

	// デバイスの最大入力チャンネル数を取得
	AioGetAiMaxChannels(id, &numData);

	//*** data ***
	//配列の大きさを確保
	/*
	ppInputData= new float*[maxSample];
	for (int i= 0; i<maxSample; i++){
	ppInputData[i] = new float[numData];
	}
	//0で初期化
	for ( int i= 0; i<maxSample; i++){
	for (int j= 0; j<numData; j++){
	ppInputData[i][j] = 0.0;
	//printf("data[%d][%d] = %f\n", i,j,data[i][j]);
	}
	}
	*/

	//*** data ***
	//配列の大きさを確保
	ppInputData = new float*[numData];
	for (int i = 0; i<numData; i++) {
		ppInputData[i] = new float[maxSample];
	}
	//0で初期化
	for (int i = 0; i<numData; i++) {
		for (int j = 0; j<maxSample; j++) {
			ppInputData[i][j] = 0.0;
			//printf("data[%d][%d] = %f\n", i,j,data[i][j]);
		}
	}

	//*** time ***
	time = new int[maxSample];
	for (int i = 0; i<maxSample; i++) {
		time[i] = 0;
	}

#endif
}

// データ入力(スレッド）開始
void CAnalogIOManager::startMeasurement()
{
	// マルチメディアタイマーの初期化

	// コンピュータが現在サポートしている精度を取得
	TIMECAPS timercaps;
	ZeroMemory(&timercaps, sizeof(timercaps));

	MMRESULT mmresult;
	UINT uPeriod;

	mmresult = timeGetDevCaps(&timercaps, sizeof(TIMECAPS));
	//    if(errorCheck(mmresult, "timeGetDevCaps()")) return;
	//    else 
	uPeriod = timercaps.wPeriodMin;

	if (uPeriod != 1)
	{
		printf("このPC環境のタイマー分解能が1msecではありません： %d\n", uPeriod);
		return;
	}

	// タイマーの分解能を1msec(==1000Hz)に設定する
	mmresult = timeBeginPeriod(1);
	//    if(errorCheck(mmresult, "timeBeginPeriod()")) return;

	// 一定時間ごとの関数の呼び出しの設定と開始
	mmresult = timeSetEvent(1, uPeriod, TimerProc, 0, TIME_PERIODIC);
	if (mmresult == NULL)
	{
		printf("timeSetEvent()が失敗しました。\n");
		return;
	}
	else timerId = (UINT)mmresult;
}

// 測定開始時間の設定
void CAnalogIOManager::setBeginTime(DWORD _beginTime)
{
	beginTime = _beginTime;
}

// 実験条件ファイルの読み込み
bool CAnalogIOManager::loadExpSequence(char *_fname, int _nTrials)
{
	FILE *fpExp;
	char fname[256];
	sprintf(fname, "%sAnalog.csv", _fname);
	printf("%s\n", fname);//check

	fpExp = fopen(fname, "r");
	if (fpExp == NULL) {
		printf("file open error!!\n");
		return false;
	}

	//試行回数の数え上げ
	int num = 0;
	char str[1500];
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		num++;
		//printf("num = %d\n",num);
	}
	nTrials = num;
	printf("nTrials = %d\n", nTrials);

	fclose(fpExp);

	// Phantomの試行回数と異なったら、エラーとして終了する
	if (_nTrials != nTrials)
	{
		printf("nTrials must be same!\nPlease check expAnalog.csv and expPhantom*.csv\n");
		exit(0);
	}

	//実験条件の読み込み
	fpExp = fopen(fname, "r");
	pnInputChannels = new int[nTrials];

	num = 1;	// todo: check why not 0?
	int tr;		// trial: 読み込むだけで保存はしない
	fgets(str, sizeof(str), fpExp);//１行目はヘッダーなので無視する
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		if (num<nTrials) {
			sscanf(str, "%d,%d",
				&tr, &pnInputChannels[num]);
			printf("pnInputChannels[%d] = %d;\n",
				num, pnInputChannels[num]);
			num++;
		}
	}

	//0番目には、1番目と同じ条件をセット // todo: check why?
	pnInputChannels[0] = pnInputChannels[1];

	fclose(fpExp);

	return true;
}

// データの入力・メモリ保存
void CAnalogIOManager::inputData()
{
	// 2009/07/17 Hirashima
	// (nTrials/2)+2試行目でinputData()が実行されなくなる。
	//printf("sample = %d \n", sample);
	//*** 原因 *** currentTrialが1,3,5と増加していた。
	//*** 解決 *** setInTrialを修正して解決！

#ifdef USE_CAIO_LIB

	// 試行中かつ保存モード中のみ
	if (bInTrial && bInSave)
	{
		// maxSampleを超えていなければ
		if (sample < maxSample)
		{
			// タイムスタンプの取得
			time[sample] = timeGetTime() - beginTime;

			// 全チャンネルについてアナログ入力データの取得
			//printf("currentTrial = %d \n", currentTrial);
			for (int i = 0; i < pnInputChannels[currentTrial]; i++)
			{
				float data;
				AioSingleAiEx(id, i, &data);
				ppInputData[i][sample] = data;
			}

			// 現在のサンプル番号をインクリメント
			sample++;
		}
	}

#endif
}

// 現在のデータをset
void CAnalogIOManager::setAnalogIO(int ch, float a)
{
#ifdef USE_CAIO_LIB

	long ret0 = AioSingleAoEx(id, ch, a);
	//long ret0 = AioSingleAiEx( id, 0, &data0 );

#endif
}

// 現在のデータをget
float CAnalogIOManager::getAnalogIO(int ch)
{
#ifdef USE_CAIO_LIB

	float a = 0;
	long ret0 = AioSingleAiEx(id, ch, &a);
	return a;

#else

	return 0.0f;

#endif
}

// データのファイル保存
void CAnalogIOManager::saveData(int trial, char* fname)
{
	char filename[500];
	sprintf(filename, "%sAnalog.csv", fname);
	FILE* fp = fopen(filename, "w");
	if (fp == NULL) {
		fprintf(stderr, "can't open %s\n", filename);
		exit(0);
	}

	///////////////////////////////////
	// 最上行の項目名

	// タイムスタンプ
	fprintf(fp, "time[msec], ");

	// 出力信号
	// todo: 現仕様では出力信号がないのでスキップ

	// 入力信号
	for (int i = 0; i < pnInputChannels[trial]; i++)
	{
		fprintf(fp, "input %d [mV]", i + 1);

		// 最後尾以外にカンマを付加
		if (i < pnInputChannels[trial] - 1)
		{
			fprintf(fp, ", ");
		}
	}
	fprintf(fp, "\n");

	///////////////////////////////////
	//時系列データ保存
	for (int i = 0; i < maxSample; i++)
	{
		// タイムスタンプ
		fprintf(fp, "%d, ", time[i]);

		// 出力信号
		// todo: 現仕様では出力信号がないのでスキップ

		// 入力信号
		for (int j = 0; j < pnInputChannels[trial]; j++)
		{
			fprintf(fp, "%f", ppInputData[j][i]);

			// 最後尾以外にカンマを付加
			if (j < pnInputChannels[trial] - 1)
			{
				fprintf(fp, ", ");
			}
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}

// 試行モードのセット
void CAnalogIOManager::setInTrial(bool flag)
{
	bInTrial = flag;

	// 試行開始に伴い、データ入力の初期化
	//flag==trueの時だけ試行回数をインクリメント 2009/07/17 Hirashima
	if (flag) {
		currentTrial++;
		sample = 0;
	}
}

// 保存モードのセット
void CAnalogIOManager::setInSave(bool flag)
{
	bInSave = flag;
}

// 設定した通り、一定時間ごとに呼ばれるタイマー用関数
#ifdef _WIN64
void CALLBACK TimerProc(UINT      uID,    // タイマーID
	UINT      uMsg,   // 予約
	DWORD_PTR dwUser, // ユーザー定義
	DWORD_PTR dw1,    // 予約
	DWORD_PTR dw2     // 予約
	)
{
	g_analogIOmanager.inputData();
}
#else
void CALLBACK TimerProc(UINT uID,     // タイマーID
	UINT uMsg,    // 予約
	DWORD dwUser, // ユーザー定義
	DWORD dw1,    // 予約
	DWORD dw2     // 予約
	)
{
	g_analogIOmanager.inputData();
}
#endif
