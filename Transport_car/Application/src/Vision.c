#include "Vision.h"
#include <stdint.h>
#include "Gimbal.h"
#include "bsp_usart.h"
#include "Chassis.h"
#include "usart.h"
#include <string.h>

Vision vision;
VisionReceive vision_receive;
VisionTransmit vision_transmit;

void Vision_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3RxBuf, sizeof(usart3RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void Vision_Receive_DataUpdate(void)
{
    if (usart3RxBuf[0] == 0xA5 && usart3RxBuf[63] == 0x5A)
	{	
		memcpy(&vision_receive, &usart3RxBuf, sizeof(vision_receive));
	}
}

void Vision_DataUpdate(void)
{
    if(vision_receive.chassis_control == 1)
    {
        chassis.pattern = Chassis_AI;
    }
    else 
    {
        chassis.pattern = Chassis_Control;
    }

    if(chassis.pattern == Chassis_AI)
    {
        chassis.move.vy = vision_receive.linear_y;
        chassis.move.vx = vision_receive.linear_x;
        chassis.move.vw = vision_receive.angular_z;
    }

    // gimbal.position.yawAngle = 
    gimbal.position.lifttHeight = vision_receive.liftHeight; 

    gimbal.carry.is_grasp_flag = vision_receive.is_grasp;
}

void Task_VisionCallback(void)
{
    Vision_DataUpdate();
}

void OS_VisionCallback(void* argument)
{
    for(;;)
    {
        Task_VisionCallback();
    }
}
