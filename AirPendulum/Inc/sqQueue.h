#ifndef __SQQUEUE_H
#define __SQQUEUE_H

#include "stm32f4xx.h"

#define SIZE 200

typedef struct{      //��ѭ������ʵ��200�㻬���˲���
	int front;       //��ͷָ��
	int rear;        //��βָ��
	int flag;        //��־,������ʱ��λ
	float queue[SIZE];
	float sum;
	float filer;
} sq_Queue;

void insert(sq_Queue *Queue, float value);
void delet(sq_Queue *Queue);
void display(sq_Queue *Queue);
float MovAve(sq_Queue *Queue);    //�����˲�

#endif



