#include "Chassis.h"
#include "cmsis_os.h"
#include <stdint.h>

#ifndef PI
#define PI 3.1415926535
#endif

Chassis chassis;

void Chassis_Init(void)
{
    chassis.info.wheeltrack = 0.3f;
    chassis.info.wheelbase = 0.3f;
    chassis.info.wheelradius = 0.03f;
}

void Chassis_Motor_Speed_Calc(void)
{
    float vx = chassis.move.vx; //左正(m/s)
    float vy = chassis.move.vy; //前正(m/s)
    float vw = chassis.move.vw; //逆时针正 (rad/s)

    float k = 0.5f * (chassis.info.wheeltrack + chassis.info.wheelbase);

    float mps_to_rpm = 60.0f / (2.0f * PI * chassis.info.wheelradius);//m/s->rpm

    float motor_rpm[4];
    motor_rpm[0] =  (vy - vx - k * vw) * mps_to_rpm; //0左前
    motor_rpm[1] = -(vy + vx + k * vw) * mps_to_rpm; //1右前
    motor_rpm[2] =  (vy + vx - k * vw) * mps_to_rpm; //2左后
    motor_rpm[3] = -(vy - vx + k * vw) * mps_to_rpm; //3右后

    for(uint8_t i = 0;i < 4;i++)
    {
        chassis.motor[i].targetSpeed = motor_rpm[i];
    }
}

void Task_ChassisCallback(void)
{
    Chassis_Motor_Speed_Calc();
}

void OS_ChassisCallback(void* argument)
{
    for(;;)
    {
        Task_ChassisCallback();
        osDelay(1);
    }  
}
