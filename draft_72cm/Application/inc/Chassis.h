#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "moto.h"

#define CHASSIS_WHEEL_NUM   4

typedef struct
{
    SingleMotor motor[CHASSIS_WHEEL_NUM];

    struct
    {
        float wheeltrack;
        float wheelbase;
        float wheelradius;      /* 实测，别信标称值 */
        float pulsesPerRev;     /* 默认 16 细分 = 3200 */
    } info;

    struct Move
    {
        float vx;
        float vy;
        float vw;               /* rad/s */
        float maxVx, maxVy, maxVw;

        float vx_real, vy_real, vw_real;
    } move;

    struct
    {
        uint8_t  active;        /* 1 = 位置模式进行中，速度帧必须屏蔽 */
        uint8_t  arrived;       /* 1 = 运动结束（到位或按时走完） */
        uint8_t  timedout;      /* 1 = 是超时结束的，不是收到 9F */
        uint8_t  error;         /* 1 = 异常：发送失败，或只有部分轮子报到位 */
        int32_t  cmdPulse[CHASSIS_WHEEL_NUM];   /* 带符号下发值 */
        uint16_t rpm;
        uint8_t  acc;
        uint32_t tStart;
        uint32_t timeoutMs;
    } pos;

    struct
    {
        float pulsesPerMM[CHASSIS_WHEEL_NUM];   /* 每轮单独标定 */
    } calib;

    enum
    {
        Chassis_Control,
        Chassis_AI,
    } pattern;                  /* Vision.c 在用，保持原语义不要动 */
} Chassis;

extern Chassis chassis;

void Chassis_Init(void);
void Chassis_Motor_Speed_Calc(void);

/* ---- 位置模式定距运动 ---- */

/* 逐轮标定：pulses_per_mm = 发出的脉冲数 / 实际走过的毫米数 */
void    Chassis_SetWheelCalib(uint8_t idx, float pulses_per_mm);

/* 非阻塞发起：立刻返回，靠 Chassis_Pos_IsArrived() 轮询 */
uint8_t Chassis_Move_Distance(float meters, uint16_t rpm, uint8_t acc,
                              uint32_t extra_timeout_ms);

/* 阻塞版本：等走完或超时才返回，1 = 正常到位，0 = 超时 */
uint8_t Chassis_Move_Distance_Wait(float meters, uint16_t rpm, uint8_t acc);

/* 1 = 已结束（到位或超时），0 = 还在走 */
uint8_t Chassis_Pos_IsArrived(void);

/* 切回速度模式并下发 0 速 */
void    Chassis_Stop(void);

#endif
