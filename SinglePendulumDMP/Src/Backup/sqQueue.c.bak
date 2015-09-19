#include "sqQueue.h"

sq_Queue Queue_q0;
sq_Queue Queue_q1;
sq_Queue Queue_q2;
sq_Queue Queue_q3;

sq_Queue Queue_gx;
sq_Queue Queue_gy;



//int queue[SIZE],reat=-1,front=-1,item;

void insert(sq_Queue *Queue, float value)
{
	if((Queue->front==0&&Queue->rear==SIZE-1)||(Queue->front==Queue->rear+1))
	{
//		printf("\n Queue is full.\n");
		Queue->flag = 1;
		Queue->sum-=Queue->queue[Queue->front];  //当队列满时，在删除队首之前需将累加值减去队首只在之后才加上队尾值
	  delet(Queue); //队列满删除队首
	}
//	else
//	{
		if(Queue->rear==-1)
		{
			Queue->rear=0;
			Queue->front=0;
		}
		else if(Queue->rear==SIZE-1)
			Queue->rear=0;
		else 
			Queue->rear++;
		Queue->queue[Queue->rear]=value;
		Queue->sum+=value;             //累加
//	}
}

void delet(sq_Queue *Queue)
{
	if(Queue->front==-1)
		printf("\n Queue is empty.\n");
	else
	{
		if(Queue->front==Queue->rear)
		{
			Queue->front=-1;
			Queue->rear=-1;
		}
		else if(Queue->front==SIZE-1)
			Queue->front=0;
		else
			Queue->front++;
	}
}

float MovAve(sq_Queue *Queue)
{
	Queue->filer = Queue->sum/SIZE;
	return Queue->filer;
}

//void display(sq_Queue *Queue)
//{
//	int i;
//	
//	if((Queue->front==-1)||(Queue->front==Queue->rear+1))
//		printf("\nQueue is empty.\n");
//	else
//	{
//		printf("n\n");
//		
//		for(i=Queue->front ;i<= Queue->rear;i++)
//			printf("\t%d",Queue->queue[i]);
//		
//	}
//}
