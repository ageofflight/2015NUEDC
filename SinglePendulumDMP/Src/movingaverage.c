#define N=500;
short value_buf[N];
char i=0;

short filter_0()
{
	char count;
	static char flag; //�궨��ֵ������Ϊһʱ������N��������ӣ�
	static int firt_value;
	static int last_value;
	int sum=0;
	short temp=0;
	value_buf[i++] = get_ad();
	if(i == N)
		i = 0;
	for(count=0;count<N;count++)
	{
		sum += value_buf[count];
	}
	temp=(short)(sum/N);
	return temp;
}
