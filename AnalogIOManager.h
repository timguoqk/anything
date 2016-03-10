#pragma once
/* CAnalogIOManager�N���X�̃w�b�_�t�@�C�� */

#pragma once

#include <windows.h>	// �A�i���O���o�̓{�[�h API �̃C���N���[�h�ɕK�v


/* CAnalogIOManager�N���X */
// �A�i���O���o�͂��Ǘ�
class CAnalogIOManager
{

	/* �R���X�g���N�^�E�f�R���X�g���N�^ */
	// �V���O���g���̂��߁Apublic�A�N�Z�X���s��Ȃ�
private:
	CAnalogIOManager(void);
	~CAnalogIOManager(void);

	/* �p�u���b�N�֐� */
public:
	// �C���X�^���X�擾�F�V���O���g���̂���
	static CAnalogIOManager &getInstance(void) {
		static CAnalogIOManager singleton;
		return singleton;
	}

	// ������
	// ���o�̓{�[�h�̏��������s���B��������true, ���s����false��Ԃ��B
	bool init();

	// �I����
	// ���o�̓{�[�h�̏I�������s���B��������true, ���s����false��Ԃ��B
	bool destroy();

	// ���̓f�[�^�z��̏�����
	void initInputData(int fs, int recordTime, int maxSample);

	// �f�[�^����(�X���b�h�j�J�n
	void startMeasurement();

	// ����J�n���Ԃ̐ݒ�
	void setBeginTime(DWORD beginTime);

	// ���������t�@�C���̓ǂݍ���
	// ���Ȃ낮���������t�@�C�����f�[�^��ǂݍ��ށB
	// fname�̈Ӗ���Phantom::saveData()�Ɠ��l�ł���B
	bool loadExpSequence(char *fname, int nTrials);

	// �f�[�^�̓��́E�������ۑ�
	void inputData();

	// �f�[�^�̃t�@�C���ۑ�
	void saveData(int trial, char* fname);

	// ���s���[�h�̃Z�b�g
	void setInTrial(bool flag);

	// �ۑ����[�h�̃Z�b�g
	void setInSave(bool flag);

	//hira�ǉ�//�o�͂̂��߂ɕK�v
	void  setAnalogIO(int ch, float a);

	//�A�i���O���͂��Q�b�g
	float getAnalogIO(int ch);

	/* �v���C�x�[�g�����o�[�ϐ� */
private:

	// ��������
	int		nTrials;					// �S���s��
	int*	pnInputChannels;			// �ۑ�������̓`�����l����[���s�񐔕��̓��I�z��]:-1��NaN�ɑ���

										// ���[�h
	bool	bInTrial;					// ���s���[�h
	bool	bInSave;					// �ۑ����[�h

	short	id;							// �f�o�C�XID
	int		currentTrial;				// ���݂̎��s�ԍ�

										// ���̓f�[�^
	float**	ppInputData;				// �A�i���O���̓f�[�^[�`�����l����][�T���v����]�i���s���Ƀ��Z�b�g�j
	int*  time;						// �A�i���O���̓^�C���X�^���v[�T���v����]�i���s���Ƀ��Z�b�g�j
	int fs;
	int recordTime;
	int maxSample;
	int sample;
	short numData;

	// ����J�n����
	int beginTime;

	// �}���`���f�B�A�^�C�}�[��ID
	UINT	timerId;
};
#define g_analogIOmanager CAnalogIOManager::getInstance()
// �V���O���g���A�N�Z�X�p�}�N��
// �A�v�����̂ǂ�����ł��ACAnalogIOManager�N���X�̗B��̃C���X�^���X�ɃO���[�o���A�N�Z�X�\
