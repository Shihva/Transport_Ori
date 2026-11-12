#include "Gimbal.h"
#include "cmsis_os2.h"

#define PULSE_PER_DEGREE 0
#define PULSE_PER_MM 0

Gimbal gimbal;

void Gimbal_Init(void)
{
    gimbal.info.minHeight = 0.0f;
    gimbal.info.maxHeight = 200.0f;

    gimbal.info.yawMotorZeroAngle = 0.0f;
    gimbal.info.liftMotorZeroAngle = 0.0f;

    gimbal.position.yawSpeed = 100.0f;
    gimbal.position.liftSpeed = 100.0f;
    gimbal.position.yawAngle = 90.0f;
    gimbal.position.lifttHeight = 0.0f;
}

void Gimbal_Angle_Calc(void)
{
    if (gimbal.position.lifttHeight > gimbal.info.maxHeight) gimbal.position.lifttHeight = gimbal.info.maxHeight;
    else if (gimbal.position.lifttHeight < gimbal.info.minHeight) gimbal.position.lifttHeight = gimbal.info.minHeight;

    float target_yaw_deg = gimbal.position.yawAngle + gimbal.info.yawMotorZeroAngle;
    gimbal.yawMotor.targetPulse  = target_yaw_deg * PULSE_PER_DEGREE;
    gimbal.yawMotor.targetSpeed  = (gimbal.position.yawSpeed > 0.0f) ? gimbal.position.yawSpeed : 100.0f; //默认100RPM

    float target_lift_mm = gimbal.position.lifttHeight + gimbal.info.liftMotorZeroAngle;
    gimbal.liftMotor.targetPulse = target_lift_mm * PULSE_PER_MM;
    gimbal.liftMotor.targetSpeed = (gimbal.position.liftSpeed > 0.0f) ? gimbal.position.liftSpeed : 100.0f; //默认100
}

void Gimbal_Carry_Update(void)
{
    
}

void Task_GimbalCallback(void)
{
    if(gimbal.carry.is_place_flag == 1)
    {
        Gimbal_Carry_Update();
    }

    Gimbal_Angle_Calc();
}

void OS_GimbalCallback(void* argument)
{
    for(;;)
    {
        Task_GimbalCallback();
        osDelay(5);
    }
}
