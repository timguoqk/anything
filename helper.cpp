// helper.cpp: 

#if defined(WIN32)
#include <windows.h>
#endif

#include <math.h>
#include "helper.h"

//*******************************
// 色
//*******************************
void getColor(int i, double color[3])
{
	if (i == 1)	memcpy(color, magenta, sizeof(magenta));
	else if (i == 2)	memcpy(color, cyan, sizeof(cyan));
	else if (i == 3)	memcpy(color, yellow, sizeof(yellow));
	else if (i == 4)	memcpy(color, red, sizeof(red));
	else if (i == 5)	memcpy(color, green, sizeof(green));
	else if (i == 6)	memcpy(color, blue, sizeof(blue));
	else if (i == 7)	memcpy(color, white, sizeof(white));
	else if (i == 8)	memcpy(color, black, sizeof(black));
	else if (i == 9)	memcpy(color, gray, sizeof(gray));
	else				memcpy(color, white, sizeof(white));

}

//*******************************
// 行列計算
//*******************************
void mul(double mat[9], double vec[3], double res[3])
{
	//vecとresに同じ配列が入ってくる場合もあるので、新たに変数を用意
	double a, b, c;
	a = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
	b = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
	c = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
	res[0] = a;
	res[1] = b;
	res[2] = c;
}
void mul36(double mat[18], double vec[6], double res[3])
{
	double a, b, c;
	a = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2] + mat[3] * vec[3] + mat[4] * vec[4] + mat[5] * vec[5];
	b = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2] + mat[9] * vec[3] + mat[10] * vec[4] + mat[11] * vec[5];
	c = mat[12] * vec[0] + mat[13] * vec[1] + mat[14] * vec[2] + mat[15] * vec[3] + mat[16] * vec[4] + mat[17] * vec[5];
	res[0] = a;
	res[1] = b;
	res[2] = c;
}
void trans(double mat[9], double res[9])
{
	res[0] = mat[0];
	res[1] = mat[3];
	res[2] = mat[6];

	res[3] = mat[1];
	res[4] = mat[4];
	res[5] = mat[7];

	res[6] = mat[2];
	res[7] = mat[5];
	res[8] = mat[8];
}
void scale(double k, double vec[3], double res[3])
{
	res[0] = k * vec[0];
	res[1] = k * vec[1];
	res[2] = k * vec[2];
}

void add(double vec1[3], double vec2[3], double res[3])
{
	double a, b, c;
	a = vec1[0] + vec2[0];
	b = vec1[1] + vec2[1];
	c = vec1[2] + vec2[2];
	res[0] = a;
	res[1] = b;
	res[2] = c;
}

void setZero(double res[3])
{
	res[0] = 0.0; res[1] = 0.0; res[2] = 0.0;
}

void setValue3(double a, double b, double c, double res[3])
{
	res[0] = a; res[1] = b; res[2] = c;
}

double norm(double vec[3])
{
	double mag;
	mag = sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]));
	return mag;
}

void tasu(double a, double b, double* c)
{
	*c = a + b;
}

//************** tasuのチェック ********************
//double a=1;double b=2;double c=5;
//tasu(a,b,&c);printf("c = %f", c);
//**************************************************

