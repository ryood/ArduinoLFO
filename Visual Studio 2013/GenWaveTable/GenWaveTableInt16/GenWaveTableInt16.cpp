// GenWaveTable.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//
// 2018.01.22 int16�^�̔g�`�e�[�u�����o��
// 

#include "stdafx.h"

#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>

#define MAX_VALUE	(4096)
#define SAMPLE_NUM	(1024)

int _tmain(int argc, _TCHAR* argv[])
{
	int i;
	double delta = (double)(MAX_VALUE - 1) / (double)SAMPLE_NUM;

	printf("/*** MAX_VALUE = %d SAMPLE_NUM = %d delta = %lf***/\n", MAX_VALUE, SAMPLE_NUM, delta);

	// �T�C���g�̐��� (0 .. MAX_VALUE-1)
	printf("/*** sine wave ***/\n");
	for (i = 0; i < SAMPLE_NUM; i++) {
		double dv = sin(2.0 * M_PI * i / SAMPLE_NUM);
		int16_t iv = (dv + 1.0) / 2.0 * (MAX_VALUE - 1);
		printf("\t%d\t,\n", iv);
	}
	printf("\n");

	// �O�p�g�̐���
	printf("/*** triangle wave ***/\n");
	double dv = (double)MAX_VALUE / 2.0;
	for (i = 0; i < SAMPLE_NUM / 4; i++) {
		dv += delta * 2.0;
		printf("\t%d\t,\n", (int)dv);
	}
	for (; i < 3 * SAMPLE_NUM / 4; i++) {
		dv -= delta * 2.0;
		printf("\t%d\t,\n", (int)dv);
	}
	for (; i < SAMPLE_NUM; i++) {
		dv += delta * 2.0;
		printf("\t%d\t,\n", (int)dv);
	}
	printf("\n");

	// ��`�g�̐���
	printf("/*** squre wave ***/\n");
	for (i = 0; i < SAMPLE_NUM / 2; i++) {
		printf("\t%d\t,\n", MAX_VALUE - 1);
	}
	for (; i < SAMPLE_NUM; i++) {
		printf("\t%d\t,\n", 0);
	}
	printf("\n");

	// �m�R�M���g�̐����i�㏸�j
	printf("/*** sawtooth upword wave ***/\n");
	for (i = 0; i < SAMPLE_NUM; i++) {
		printf("\t%d\t,\n", (int)(i * delta));
	}
	printf("\n");

	// �m�R�M���g�̐����i���~�j
	printf("/*** sawtooth downword wave ***/\n");
	for (i = SAMPLE_NUM - 1; i >= 0; i--) {
		printf("\t%d\t,\n", (int)(i * delta));
	}

	return 0;
}

