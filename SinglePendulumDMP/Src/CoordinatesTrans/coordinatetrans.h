#ifndef __COORDINATETRANS_H
#define __COORDINATETRANS_H

#include <math.h>

#define pi 3.14159265359f
#define Length 100       //¸Ë³¤1m

typedef struct{
	double qw;
	double qx;
	double qy;
	double qz;
} Quaternion;

typedef struct{
	double angle;
	double ax;
	double ay;
	double az;
} AxisAngle;

typedef struct
{
	double vx;
	double vy;
	double vz;
} Vector;

typedef struct
{
	Vector mx;
	Vector my;
	Vector mz;
} Matrix3_3;

typedef struct{
	float x;
	float y;
	float z;
	float Ypsino;
	float Sigma;
	float Alpha;
}Position3D;

void Quaternion2AxisAngel(Quaternion* q, AxisAngle* AA);
void AxisAngle2Quaternion(AxisAngle* AA, Quaternion* q);

void Coordinate_Trans(float pitch, float roll, float yaw, Position3D* coordi);



#endif

