#include "Moto.h"
#include "Gimbal.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "usart.h"

uint8_t last_gimbal_ctrl_time = 0;

void Task_MotorCallback(void)
{
    set_chassis_ctrl(&huart1, chassis.motor[0].targetSpeed, chassis.motor[1].targetSpeed, chassis.motor[2].targetSpeed, chassis.motor[3].targetSpeed);
    if(++ last_gimbal_ctrl_time >= 20)
    {
        last_gimbal_ctrl_time = 0;
        set_gimbal_ctrl(&huart2, gimbal.yawMotor.targetPulse, gimbal.liftMotor.targetPulse);
    }
}

void OS_MotorCallback(void* argument)
{
    for(;;)
    {
        Task_MotorCallback();
        osDelay(1);
    }
}
