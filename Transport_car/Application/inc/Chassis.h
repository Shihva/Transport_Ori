#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "moto.h"

typedef struct
{
    SingleMotor motor[4];

    struct 
	{
		float wheeltrack;
        float wheelbase;
		float wheelradius;
	} info;

    struct Move
	{
		float vx;
		float vy;
		float vw;//rad/s
		float maxVx, maxVy, maxVw;//Vw rad/s

		float vx_real, vy_real, vw_real;
	} move;

	enum
	{
		Chassis_Control,
		Chassis_AI,
	} pattern;
}Chassis;

extern Chassis chassis;

void Chassis_Init(void);
void Chassis_Motor_Speed_Calc(void);

#endif
