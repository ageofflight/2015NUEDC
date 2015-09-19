#include "coordinatetrans.h"


/** 
  ****************************************************************************************************************************
	*@attention
	* Program was refered from  http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/          *
  * and http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/. When using function           *
	* void Quaternion2AxisAngel(Quaternion q, AxisAngle AA), one must know that it may cause error because of not              *
  * not considering the existent of Sigularities at angle = 0 degrees and 180 degrees. Please refer to the pdf file included *
  * in the source file	                                                                                                     *
	****************************************************************************************************************************
	*@author ageofflight
	*@data   21-July-2015
	*@brief  coordinate transformation
	*/

/* Quaternion To AxisAngle */
void Quaternion2AxisAngel(Quaternion* q, AxisAngle* AA)
{
	float _temp=0.0f;
	_temp = 1.0f/sqrt(1.0f-q->qw*q->qw);
	AA->angle = 2*acos(q->qw);
	AA->ax = q->qx * _temp;
	AA->ay = q->qy * _temp;
	AA->az = q->qz * _temp;
}

/* AxisAngle To Quaternion */
void AxisAngle2Quaternion(AxisAngle* AA, Quaternion* q)
{
	float _temp=0;
	_temp = sin(AA->angle/2);
	q->qx = AA->ax * _temp;
	q->qy = AA->ay * _temp;
	q->qz = AA->az * _temp;
	q->qw = cos(AA->angle/2);
}

void Quaternion2Matrix(Quaternion* q, Vector* v)
{
	Matrix3_3 matrix;
	Vector temp;
	
	matrix.mx.vx = 1-2*q->qy*q->qy-2*q->qz*q->qz;
	matrix.mx.vy = 2*q->qx*q->qy-2*q->qz*q->qw;
	matrix.mx.vx = 2*q->qx*q->qz+2*q->qy*q->qw;
	
	matrix.my.vx = 2*q->qx*q->qy+2*q->qz*q->qw;
	matrix.my.vy = 1-2*q->qx*q->qx-2*q->qz*q->qz;
	matrix.my.vz = 2*q->qy*q->qz-2*q->qx*q->qw;
	
	matrix.mz.vx = 2*q->qx*q->qz-2*q->qy*q->qw;
	matrix.mz.vy = 2*q->qy*q->qz+2*q->qx*q->qw;
	matrix.mz.vz = 1-2*q->qx*q->qx-2*q->qy*q->qy;
	
	temp.vx = matrix.mx.vx*v->vx+matrix.mx.vy*v->vy+matrix.mx.vz*v->vz;
	temp.vy = matrix.my.vx*v->vx+matrix.my.vy*v->vy+matrix.my.vz*v->vz;
	temp.vz = matrix.mz.vx*v->vx+matrix.mz.vy*v->vy+matrix.mz.vz*v->vz;
	
	v->vx = temp.vx;
	v->vy = temp.vy;
	v->vz = temp.vz;
}

/*由自己计算的由欧拉角到轴角的转换公式，未试验过正确性*/
void Coordinate_Trans(float pitch, float roll, float yaw, Position3D* coordi)
{
	float Temp_TanPhi, Temp_CosTheta, Temp_CosPhi, Temp_SinPhi, Temp_A, Temp_B, Temp_C, Temp_D, Temp_E;
	Temp_TanPhi=tan(pi/2-roll);
	Temp_CosTheta = cos(pitch);
	Temp_CosPhi = cos(roll);
	Temp_SinPhi = sin(roll);
	Temp_A=sqrt(Temp_TanPhi*Temp_TanPhi*Temp_CosTheta*Temp_CosTheta+1);
	Temp_B=acos(1/Temp_A);
	Temp_C=cos(yaw-pi/2-Temp_B);
	
	Temp_D=sqrt(Temp_CosPhi*Temp_CosPhi*Temp_CosTheta*Temp_CosTheta+Temp_CosTheta*Temp_CosTheta);
	Temp_E=atan2(Temp_CosPhi*Temp_CosTheta, Temp_SinPhi);
	
	
	float Temp_CosYpsino, Temp_CosSigma, Temp_CosAlpha;
//	float Ypsino, Sigma, Alpha;
	
	Temp_CosYpsino = Temp_A*Temp_C;
	Temp_CosSigma = Temp_D*cos(Temp_E+pi-yaw);
	Temp_CosAlpha = Temp_CosTheta*Temp_CosTheta*Temp_CosPhi;
	
	coordi->Ypsino = acos(Temp_CosYpsino);
	coordi->Sigma = acos(Temp_CosSigma);
	coordi->Alpha = acos(Temp_CosAlpha);
	
	coordi->x=Length*Temp_CosYpsino;  //杆长在X轴的投影
	coordi->y=Length*Temp_CosSigma;   //杆长在Y轴的投影
}




