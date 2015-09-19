#include "filter.h"


/**
  * n-���α�����״̬������ά����m-���ͱ�������̬������ά����len-���ͱ������۲����еĳ��ȡ�f-˫����ʵ�Ͷ�ά���飬���Ϊn��n��ϵͳ״̬ת�ƾ���phi(k,k-1)
	* d-˫����ʵ����Ϊ���飬����Ϊn��ϵͳ�����źŵ�ϵ������Dk��u-˫����ʵ�ͱ�����ϵͳ�������ź�Vk��b-˫����ʵ�Ͷ�ά���飬���Ϊn��m����̬������ϵ������Bk��
	* q-˫����ʵ�Ͷ�ά���飬���Ϊm��m,��̬������Э�������Qk��r-˫����ʵ�ͱ������۲�������Э����Rk��z-˫����ʵ��һά���飬����Ϊlen��ϵͳ�Ĺ۲�ֵZk��
	* x-˫����ʵ��һά���飬����Ϊn��ϵͳ��״̬����Xk��p-˫����ʵ�Ͷ�ά���飬���Ϊn��n��ϵͳ���˲�������Pk�� 
	* g-˫����ʵ��һά���飬����Ϊn��ϵͳ�Ŀ������˲�����Kk��
	* 
  */
void kalman(int n, int m, int len, double f[], double d[], double u[], double b[], double q[], double h[], double r, double z[], double x[], double p[], double g[])
{
	int i,j,k,k1;
	double y, *p1, *c, *s, *cc, *x1;
	p1 = malloc(n*n*sizeof(double));
	c = malloc(n*n*sizeof(double));
	s = malloc(n*n*sizeof(double));
	x1 = malloc(n*sizeof(double));
	cc = malloc(n*sizeof(double));
	for(k1=0;k1<len;k1++)
	{
		for(i=0;i<n;i++)
		{
			y=0.0f;
			for(j=0;j<n;j++)
			{
				y=y+f[i*n+j]*x[j];
			}
			x1[i]=y+d[i]*u[k1];
		}
		for(i=0;i<n;i++)
		{
			for(j=0;j<n;j++)
			{
				y=0.0f;
				for(k=0;k<n;k++)
				{
					y=y+f[i*n+k]*p[k*n+j];
				}
				c[i*n+j]=y;
			}
		}
		for(i=0;i<n;i++)
		{
			for(j=0;j<n;j++)
			{
				y=0.0f;
				for(k=0;k<n;k++)
				{
					y=y+c[i*n+k]*f[j*n+k];
				}
				p1[i*n+j]=y;
			}
		}
		for(i=0;i<n;i++)
		{
			for(j=0;j<m;j++)
			{
				y=0.0f;
				for(k=0;k<m;k++)
				{
					y=y+b[i*m+k]*q[k*m+j];
				}
				s[i*m+j]=y;
			}
		}
		for(i=0;i<n;i++)
		{
			for(j=0;j<n;j++)
			{
				y=0.0f;
				for(k=0;k<m;k++)
				{
					y=y+s[i*m+k]*b[j*m+k];
				}
				p1[i*n+j]=y+p1[i*n+j];
			}
		}
		for(i=0;i<n;i++)
		{
			y=0.0f;
			for(k=0;k<n;k++)
			{
				y=y+p1[i*n+k]*h[k];
			}
			cc[i]=y;
		}
		y=0.0f;
		for(i=0;i<n;i++)
		{
			y=y+cc[i]*h[i];
		}
		y=r+y;
		for(i=0;i<n;i++)
		{
			g[i]=cc[i]/y;
		}
		for(i=0;i<n;i++)
		{
			for(j=0;j<n;j++)
			{
				p[i*n+j]=p1[i*n+j]-g[i]*cc[j];
			}
		}
		y=0.0f;
		for(i=0;i<n;i++)
		{
			y=y+h[j]*x1[i];
		}
		y=z[k1]-y;
		for(i=0;i<n;i++)
		{
			x[i]=x[i]+g[i]*y;
		}
	}
	free(p1);
	free(c);
	free(s);
	free(x1);
	free(cc);
}


