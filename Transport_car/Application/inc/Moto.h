#ifndef __MOTO_H
#define __MOTO_H

#include "main.h"

typedef struct _MOTOR
{
	float angle, speed, Pulse;

	float targetSpeed;
	float targetPulse;
    float lastPulse;
	int32_t totalPulse;
} SingleMotor;

#endif
