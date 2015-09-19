#ifndef __SQQUEUE_H
#define __SQQUEUE_H

#include "stm32f4xx.h"

#define SIZE 200

typedef struct{      //用循环队列实现200点滑动滤波器
	int front;       //排头指针
	int rear;        //队尾指针
	int flag;        //标志,队列满时置位
	float queue[SIZE];
	float sum;
	float filer;
} sq_Queue;

void insert(sq_Queue *Queue, float value);
void delet(sq_Queue *Queue);
void display(sq_Queue *Queue);
float MovAve(sq_Queue *Queue);    //滑动滤波

#endif



