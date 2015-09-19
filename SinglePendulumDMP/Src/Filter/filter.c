#include "filter.h"


/**
  * n-整形变量，状态变量的维数。m-整型变量，动态噪声的维数。len-整型变量，观测序列的长度。f-双精度实型二维数组，体积为n×n，系统状态转移矩阵phi(k,k-1)
	* d-双精度实型以为数组，长度为n。系统输入信号的系数向量Dk。u-双精度实型变量，系统的输入信号Vk。b-双精度实型二维数组，体积为n×m，动态噪声的系数矩阵Bk。
	* q-双精度实型二维数组，体积为m×m,动态噪声的协方差矩阵Qk。r-双精度实型变量。观测噪声的协方差Rk。z-双精度实型一维数组，长度为len，系统的观测值Zk。
	* x-双精度实型一维数组，长度为n，系统的状态变量Xk。p-双精度实型二维数组，体积为n×n，系统的滤波误差方差阵Pk。 
	* g-双精度实型一维数组，长度为n，系统的卡尔曼滤波增益Kk。
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


