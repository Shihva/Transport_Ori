#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "main.h"
#include "Moto.h"
#include <stdint.h>

typedef struct _GIMBAL
{
    SingleMotor liftMotor, yawMotor;

    struct
	{
        float yawMotorZeroAngle;
        float liftMotorZeroAngle;	

		float maxHeight;
		float minHeight;
	} info;

    struct Position
    {
        float lifttHeight;
        float yawAngle;
        float liftSpeed;
        float yawSpeed;

        float liftHeight_real, yawAngle_real;
        float liftspeed_real, yawspeed_real;
	} position;

    struct
    {
        uint8_t is_grasp_flag;
        uint8_t is_place_flag;
    }carry;
}Gimbal;

extern Gimbal gimbal;

#endif
