#pragma once
/* CAnalogIOManagerクラスのヘッダファイル */

#pragma once

#include <windows.h>	// アナログ入出力ボード API のインクルードに必要


/* CAnalogIOManagerクラス */
// アナログ入出力を管理
class CAnalogIOManager
{

	/* コンストラクタ・デコンストラクタ */
	// シングルトンのため、publicアクセスを行わない
private:
	CAnalogIOManager(void);
	~CAnalogIOManager(void);

	/* パブリック関数 */
public:
	// インスタンス取得：シングルトンのため
	static CAnalogIOManager &getInstance(void) {
		static CAnalogIOManager singleton;
		return singleton;
	}

	// 初期化
	// 入出力ボードの初期化を行う。成功時にtrue, 失敗時にfalseを返す。
	bool init();

	// 終了化
	// 入出力ボードの終了化を行う。成功時にtrue, 失敗時にfalseを返す。
	bool destroy();

	// 入力データ配列の初期化
	void initInputData(int fs, int recordTime, int maxSample);

	// データ入力(スレッド）開始
	void startMeasurement();

	// 測定開始時間の設定
	void setBeginTime(DWORD beginTime);

	// 実験条件ファイルの読み込み
	// あなろぐ実験条件ファイルよりデータを読み込む。
	// fnameの意味はPhantom::saveData()と同様である。
	bool loadExpSequence(char *fname, int nTrials);

	// データの入力・メモリ保存
	void inputData();

	// データのファイル保存
	void saveData(int trial, char* fname);

	// 試行モードのセット
	void setInTrial(bool flag);

	// 保存モードのセット
	void setInSave(bool flag);

	//hira追加//出力のために必要
	void  setAnalogIO(int ch, float a);

	//アナログ入力をゲット
	float getAnalogIO(int ch);

	/* プライベートメンバー変数 */
private:

	// 実験条件
	int		nTrials;					// 全試行回数
	int*	pnInputChannels;			// 保存する入力チャンネル数[試行回数分の動的配列]:-1はNaNに相当

										// モード
	bool	bInTrial;					// 試行モード
	bool	bInSave;					// 保存モード

	short	id;							// デバイスID
	int		currentTrial;				// 現在の試行番号

										// 入力データ
	float**	ppInputData;				// アナログ入力データ[チャンネル数][サンプル数]（試行毎にリセット）
	int*  time;						// アナログ入力タイムスタンプ[サンプル数]（試行毎にリセット）
	int fs;
	int recordTime;
	int maxSample;
	int sample;
	short numData;

	// 測定開始時刻
	int beginTime;

	// マルチメディアタイマーのID
	UINT	timerId;
};
#define g_analogIOmanager CAnalogIOManager::getInstance()
// シングルトンアクセス用マクロ
// アプリ中のどこからでも、CAnalogIOManagerクラスの唯一のインスタンスにグローバルアクセス可能
