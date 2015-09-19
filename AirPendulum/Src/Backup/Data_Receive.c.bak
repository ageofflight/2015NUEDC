#include "Data_Receive.h"

T_RC_DATA Rc_Data;//1000~2000
float RC_Target_ROL=0,RC_Target_PIT=0,RC_Target_YAW=0;
uint8_t ARMED = 0;

//void Rc_DataAnl(void)
//{
//	Rc_Data.THROTTLE	=	Rc_Pwm_In[9];
//	Rc_Data.YAW				=	Rc_Pwm_In[8];
//	Rc_Data.ROLL			=	Rc_Pwm_In[7];
//	Rc_Data.PITCH			=	Rc_Pwm_In[6];
//	Rc_Data.AUX1			=	Rc_Pwm_In[1];
//	Rc_Data.AUX2			=	Rc_Pwm_In[0];
//	Rc_Data.AUX3			=	Rc_Pwm_In[5];
//	Rc_Data.AUX4			=	Rc_Pwm_In[2];
//	Rc_Data.AUX5			=	Rc_Pwm_In[3];
//	Rc_Data.AUX6			=	Rc_Pwm_In[4];
//}
void Rc_DataCal(void)
{
	RC_Target_ROL = (Rc_Data.ROLL-1500)/30;
	RC_Target_PIT = (Rc_Data.PITCH-1500)/30;
	RC_Target_YAW = (Rc_Data.YAW-1500)/30;
}


