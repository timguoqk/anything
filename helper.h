#pragma once
// helper.h: 

#if !defined(HELPER_H_INCLUDED_)
#define HELPER_H_INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//*******************************
// よく使う定数
//*******************************
#define kPI 3.1415926535897932384626433832795

//*******************************
// 色
//*******************************
enum {
	MAGENTA = 1,  //MAGENTA = 1;
	CYAN,			//CYAN    = 2;
	YELLOW,
	RED,
	GREEN,
	BLUE,
	WHITE,
	BLACK,
	GRAY
};

const double magenta[3] = { 0.6,0,0.6 };
const double cyan[3] = { 0,0.6,0.6 };
const double yellow[3] = { 0.6,0.6,0 };
const double red[3] = { 0.6,0,0 };
const double green[3] = { 0,0.6,0 };
const double blue[3] = { 0,0,0.6 };
const double white[3] = { 0.7,0.7,0.7 };
const double black[3] = { 0,0,0 };
const double gray[3] = { 0.4,0.4,0.4 };
void getColor(int i, double color[3]);

//*******************************
// 行列計算
//*******************************
void mul(double mat[9], double vec[3], double res[3]);
void mul36(double mat[18], double vec[6], double res[3]);
void trans(double mat[9], double res[9]);
void scale(double k, double vec[3], double res[3]);
void add(double vec1[3], double vec2[3], double res[3]);
void setZero(double res[3]);
void setValue3(double a, double b, double c, double res[3]);
double norm(double vec[3]);

void tasu(double a, double b, double* c);

#endif // !defined(HELPER_H_INCLUDED_)
