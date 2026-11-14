#include "Moto.h"
#include "Gimbal.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "usart.h"

uint8_t last_gimbal_ctrl_time = 0;

void Task_MotorCallback(void)
{
    /* 位置模式（定距运动）进行中，绝对不能发速度帧：
     * 手册 5.1.3 明确说明位置模式命令可以被新命令打断，
     * 1ms 后到来的速度帧会把位置运动直接顶掉 */
    if (!chassis.pos.active)
    {
        set_chassis_ctrl(&huart1,
                         chassis.motor[0].targetSpeed,
                         chassis.motor[1].targetSpeed,
                         chassis.motor[2].targetSpeed,
                         chassis.motor[3].targetSpeed);
    }

    if (++last_gimbal_ctrl_time >= 20)
    {
        last_gimbal_ctrl_time = 0;
        set_gimbal_ctrl(&huart2, gimbal.yawMotor.targetPulse, gimbal.liftMotor.targetPulse);
    }
}

void OS_MotorCallback(void *argument)
{
    for (;;)
    {
        Task_MotorCallback();
        osDelay(1);
    }
}
