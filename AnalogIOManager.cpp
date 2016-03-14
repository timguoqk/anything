// �C���N���[�h
#include <stdio.h>
#include "AnalogIOManager.h"	// CAnalogIOManager�N���X�w�b�_

//#define USE_CAIO_LIB //���̂悤��define����΁Acontec board �𗘗p���郂�[�h�ɂȂ邱�Ƃ��m�F. Hirashima 2014/03/20.
// Declare include header and linking library for using contec DA board depend on the preprocessor macro.
#ifdef USE_CAIO_LIB
#include "Caio.h"					// �A�i���O���o�̓{�[�h�p API �w�b�_�̃C���N���[�h
#pragma comment( lib, "Caio.lib" )	// �A�i���O���o�̓{�[�h���C�u�����̃����N
#endif


// �萔�̒�`

// ���[�J���֐���`

// �A�i���O���o�̓^�C�}�[�֐�
#ifdef _WIN64
void CALLBACK TimerProc(UINT      uID,    // �^�C�}�[ID
	UINT      uMsg,   // �\��
	DWORD_PTR dwUser, // ���[�U�[��`
	DWORD_PTR dw1,    // �\��
	DWORD_PTR dw2     // �\��
	);
#else
void CALLBACK TimerProc(UINT uID,     // �^�C�}�[ID
	UINT uMsg,    // �\��
	DWORD dwUser, // ���[�U�[��`
	DWORD dw1,    // �\��
	DWORD dw2     // �\��
	);
#endif

// �R���X�g���N�^
// �e��ϐ��̏�����
CAnalogIOManager::CAnalogIOManager(void)
{
	currentTrial = 0;
	bInTrial = false;
	bInSave = false;
}

// �f�X�g���N�^ */
// �e��|�C���^�ϐ��̏I����
CAnalogIOManager::~CAnalogIOManager(void)
{
}

// ������
bool CAnalogIOManager::init()
{

#ifdef USE_CAIO_LIB

	long ret = AioInit("AIO000", &id);

	// �A�i���O���o�̓{�[�h�̏�����
	if (ret != 0)
	{
		printf("AioInit�ŃG���[���������܂��� %d\n", ret);
		return false;
	}

	// �����W�̐ݒ�
	AioSetAiRangeAll(id, PM10); // //�}10V

	return true;

#else

	return false;

#endif
}

// �I����
bool CAnalogIOManager::destroy()
{

#ifdef USE_CAIO_LIB

	long ret = AioExit(id);

	// �A�i���O���o�̓{�[�h�̏I����
	if (ret != 0)
	{
		printf("AioExit�ŃG���[���������܂��� %d\n", ret);
		return false;
	}

	// �}���`���f�B�A�^�C�}�[�̏I����
	timeKillEvent(timerId);
	timeEndPeriod(1);

	return true;

#else

	return false;

#endif
}

// ���̓f�[�^�z��̏�����
void CAnalogIOManager::initInputData(int _fs, int _recordTime, int _maxSample)
{
#ifdef USE_CAIO_LIB

	fs = _fs;
	recordTime = _recordTime;
	maxSample = _maxSample;

	// �f�o�C�X�̍ő���̓`�����l�������擾
	AioGetAiMaxChannels(id, &numData);

	//*** data ***
	//�z��̑傫�����m��
	/*
	ppInputData= new float*[maxSample];
	for (int i= 0; i<maxSample; i++){
	ppInputData[i] = new float[numData];
	}
	//0�ŏ�����
	for ( int i= 0; i<maxSample; i++){
	for (int j= 0; j<numData; j++){
	ppInputData[i][j] = 0.0;
	//printf("data[%d][%d] = %f\n", i,j,data[i][j]);
	}
	}
	*/

	//*** data ***
	//�z��̑傫�����m��
	ppInputData = new float*[numData];
	for (int i = 0; i<numData; i++) {
		ppInputData[i] = new float[maxSample];
	}
	//0�ŏ�����
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

// �f�[�^����(�X���b�h�j�J�n
void CAnalogIOManager::startMeasurement()
{
	// �}���`���f�B�A�^�C�}�[�̏�����

	// �R���s���[�^�����݃T�|�[�g���Ă��鐸�x���擾
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
		printf("����PC���̃^�C�}�[����\��1msec�ł͂���܂���F %d\n", uPeriod);
		return;
	}

	// �^�C�}�[�̕���\��1msec(==1000Hz)�ɐݒ肷��
	mmresult = timeBeginPeriod(1);
	//    if(errorCheck(mmresult, "timeBeginPeriod()")) return;

	// ��莞�Ԃ��Ƃ̊֐��̌Ăяo���̐ݒ�ƊJ�n
	mmresult = timeSetEvent(1, uPeriod, TimerProc, 0, TIME_PERIODIC);
	if (mmresult == NULL)
	{
		printf("timeSetEvent()�����s���܂����B\n");
		return;
	}
	else timerId = (UINT)mmresult;
}

// ����J�n���Ԃ̐ݒ�
void CAnalogIOManager::setBeginTime(DWORD _beginTime)
{
	beginTime = _beginTime;
}

// ���������t�@�C���̓ǂݍ���
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

	//���s�񐔂̐����グ
	int num = 0;
	char str[1500];
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		num++;
		//printf("num = %d\n",num);
	}
	nTrials = num;
	printf("nTrials = %d\n", nTrials);

	fclose(fpExp);

	// Phantom�̎��s�񐔂ƈقȂ�����A�G���[�Ƃ��ďI������
	if (_nTrials != nTrials)
	{
		printf("nTrials must be same!\nPlease check expAnalog.csv and expPhantom*.csv\n");
		exit(0);
	}

	//���������̓ǂݍ���
	fpExp = fopen(fname, "r");
	pnInputChannels = new int[nTrials];

	num = 1;	// todo: check why not 0?
	int tr;		// trial: �ǂݍ��ނ����ŕۑ��͂��Ȃ�
	fgets(str, sizeof(str), fpExp);//�P�s�ڂ̓w�b�_�[�Ȃ̂Ŗ�������
	while (fgets(str, sizeof(str), fpExp) != NULL) {
		if (num<nTrials) {
			sscanf(str, "%d,%d",
				&tr, &pnInputChannels[num]);
			printf("pnInputChannels[%d] = %d;\n",
				num, pnInputChannels[num]);
			num++;
		}
	}

	//0�Ԗڂɂ́A1�ԖڂƓ����������Z�b�g // todo: check why?
	pnInputChannels[0] = pnInputChannels[1];

	fclose(fpExp);

	return true;
}

// �f�[�^�̓��́E�������ۑ�
void CAnalogIOManager::inputData()
{
	// 2009/07/17 Hirashima
	// (nTrials/2)+2���s�ڂ�inputData()�����s����Ȃ��Ȃ�B
	//printf("sample = %d \n", sample);
	//*** ���� *** currentTrial��1,3,5�Ƒ������Ă����B
	//*** ���� *** setInTrial���C�����ĉ����I

#ifdef USE_CAIO_LIB

	// ���s�����ۑ����[�h���̂�
	if (bInTrial && bInSave)
	{
		// maxSample�𒴂��Ă��Ȃ����
		if (sample < maxSample)
		{
			// �^�C���X�^���v�̎擾
			time[sample] = timeGetTime() - beginTime;

			// �S�`�����l���ɂ��ăA�i���O���̓f�[�^�̎擾
			//printf("currentTrial = %d \n", currentTrial);
			for (int i = 0; i < pnInputChannels[currentTrial]; i++)
			{
				float data;
				AioSingleAiEx(id, i, &data);
				ppInputData[i][sample] = data;
			}

			// ���݂̃T���v���ԍ����C���N�������g
			sample++;
		}
	}

#endif
}

// ���݂̃f�[�^��set
void CAnalogIOManager::setAnalogIO(int ch, float a)
{
#ifdef USE_CAIO_LIB

	long ret0 = AioSingleAoEx(id, ch, a);
	//long ret0 = AioSingleAiEx( id, 0, &data0 );

#endif
}

// ���݂̃f�[�^��get
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

// �f�[�^�̃t�@�C���ۑ�
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
	// �ŏ�s�̍��ږ�

	// �^�C���X�^���v
	fprintf(fp, "time[msec], ");

	// �o�͐M��
	// todo: ���d�l�ł͏o�͐M�����Ȃ��̂ŃX�L�b�v

	// ���͐M��
	for (int i = 0; i < pnInputChannels[trial]; i++)
	{
		fprintf(fp, "input %d [mV]", i + 1);

		// �Ō���ȊO�ɃJ���}��t��
		if (i < pnInputChannels[trial] - 1)
		{
			fprintf(fp, ", ");
		}
	}
	fprintf(fp, "\n");

	///////////////////////////////////
	//���n��f�[�^�ۑ�
	for (int i = 0; i < maxSample; i++)
	{
		// �^�C���X�^���v
		fprintf(fp, "%d, ", time[i]);

		// �o�͐M��
		// todo: ���d�l�ł͏o�͐M�����Ȃ��̂ŃX�L�b�v

		// ���͐M��
		for (int j = 0; j < pnInputChannels[trial]; j++)
		{
			fprintf(fp, "%f", ppInputData[j][i]);

			// �Ō���ȊO�ɃJ���}��t��
			if (j < pnInputChannels[trial] - 1)
			{
				fprintf(fp, ", ");
			}
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}

// ���s���[�h�̃Z�b�g
void CAnalogIOManager::setInTrial(bool flag)
{
	bInTrial = flag;

	// ���s�J�n�ɔ����A�f�[�^���͂̏�����
	//flag==true�̎��������s�񐔂��C���N�������g 2009/07/17 Hirashima
	if (flag) {
		currentTrial++;
		sample = 0;
	}
}

// �ۑ����[�h�̃Z�b�g
void CAnalogIOManager::setInSave(bool flag)
{
	bInSave = flag;
}

// �ݒ肵���ʂ�A��莞�Ԃ��ƂɌĂ΂��^�C�}�[�p�֐�
#ifdef _WIN64
void CALLBACK TimerProc(UINT      uID,    // �^�C�}�[ID
	UINT      uMsg,   // �\��
	DWORD_PTR dwUser, // ���[�U�[��`
	DWORD_PTR dw1,    // �\��
	DWORD_PTR dw2     // �\��
	)
{
	g_analogIOmanager.inputData();
}
#else
void CALLBACK TimerProc(UINT uID,     // �^�C�}�[ID
	UINT uMsg,    // �\��
	DWORD dwUser, // ���[�U�[��`
	DWORD dw1,    // �\��
	DWORD dw2     // �\��
	)
{
	g_analogIOmanager.inputData();
}
#endif
