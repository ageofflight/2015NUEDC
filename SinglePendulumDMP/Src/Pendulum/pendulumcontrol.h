#ifndef __PENDULUMCONTROL_H
#define __PENDULUMCONTROL_H

#include "stm32f4xx.h"
#include "PID_Control.h"
#include "motor.h"
//#include "myiic.h"
#include <math.h>
#include "mpu6050.h"

typedef enum 
{
  PlusDirection = 0, 
  MinusDirection = !PlusDirection
} SwingDirection;

//???
typedef struct int16_xyz
{
    int16_t X;
    int16_t Y;
    int16_t Z;
}S_INT16_XYZ;

//IMU
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
    
}S_FLOAT_XYZ;

typedef struct float_angle
{
    float Roll;
    float Pitch;
    float Yaw;
}S_FLOAT_ANGLE;

#define Still 0
#define FreeFall 1
#define eAmplitude 2
#define NDirection 3
#define Circle 4
#define Eight 5

void pendulum_control(void);
void Get_ChangRate(float* Pitch, float* Roll, float* Pitch_Der, float* Roll_Der);
void pendulum_control_EPeriod(void);
void pendulum_control_EAmplitude(void);

void pendulum_control_freefall(float pitch);
void pendulum_control_eAmplitude_swing(float amplitude);
void pendulum_control_circle(float radius);
void pendulum_control_eight(float radius);
void pendulum_control_nDirection_swing(float direction, float amplitude);
void pendulum_control_Still(void);

#endif
