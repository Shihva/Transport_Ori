#include "Chassis.h"
#include "bsp_usart.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.1415926535f
#endif

Chassis chassis;

/* ==================== 初始化 ==================== */

void Chassis_Init(void)
{
    chassis.info.wheeltrack   = 0.300f;
    chassis.info.wheelbase    = 0.300f;
    chassis.info.wheelradius  = 0.030f;      /* TODO: 用滚动法实测后填写 */
    chassis.info.pulsesPerRev = 3200.0f;     /* 16 细分 */

    /* 理论脉冲当量：3200 / (2π×0.03×1000) ≈ 16.98 脉冲/mm，仅作标定起点。
     * 注意 wheelradius 单位是米，×1000 换成毫米，别漏 */
    float theory = chassis.info.pulsesPerRev /
                   (2.0f * PI * chassis.info.wheelradius * 1000.0f);

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis.calib.pulsesPerMM[i] = theory;
        chassis.motor[i].targetSpeed = 0.0f;
        chassis.motor[i].targetPulse = 0.0f;
    }

    chassis.move.vx = chassis.move.vy = chassis.move.vw = 0.0f;

    chassis.pos.active    = 0;
    chassis.pos.arrived   = 0;
    chassis.pos.timedout  = 0;
    chassis.pos.error     = 0;
    chassis.pos.timeoutMs = 0;
    chassis.pos.tStart    = 0;

    chassis.pattern = Chassis_Control;

    /* 打开 USART1 接收，否则收不到 FD 9F 到位返回 / 3A 状态 */
    Bsp_Usart1_StartRx();
}

void Chassis_SetWheelCalib(uint8_t idx, float pulses_per_mm)
{
    if (idx < CHASSIS_WHEEL_NUM && pulses_per_mm > 1.0f && pulses_per_mm < 1000.0f)
    {
        chassis.calib.pulsesPerMM[idx] = pulses_per_mm;
    }
}

/* ==================== 速度模式（麦轮逆解） ==================== */

void Chassis_Motor_Speed_Calc(void)
{
    float vx = chassis.move.vx;   /* 左正 (m/s) */
    float vy = chassis.move.vy;   /* 前正 (m/s) */
    float vw = chassis.move.vw;   /* 逆时针正 (rad/s) */

    float k = 0.5f * (chassis.info.wheeltrack + chassis.info.wheelbase);
    float mps_to_rpm = 60.0f / (2.0f * PI * chassis.info.wheelradius);

    float motor_rpm[CHASSIS_WHEEL_NUM];
    motor_rpm[0] =  (vy - vx - k * vw) * mps_to_rpm;   /* 0 左前 */
    motor_rpm[1] = -(vy + vx + k * vw) * mps_to_rpm;   /* 1 右前 */
    motor_rpm[2] =  (vy + vx - k * vw) * mps_to_rpm;   /* 2 左后 */
    motor_rpm[3] = -(vy - vx + k * vw) * mps_to_rpm;   /* 3 右后 */

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis.motor[i].targetSpeed = motor_rpm[i];
    }
}

void Task_ChassisCallback(void)
{
    if (chassis.pos.active)     /* 位置模式下不要覆盖 targetSpeed */
    {
        return;
    }
    Chassis_Motor_Speed_Calc();
}

void OS_ChassisCallback(void *argument)
{
    for (;;)
    {
        Task_ChassisCallback();
        osDelay(1);
    }
}

/* ==================== 位置模式定距运动 ==================== */

/* 加速度档位 → RPM/s（手册 5.3.12：每 (256-acc)*50us 变化 1RPM） */
static float Zdt_AccToRpmPerSec(uint8_t acc)
{
    if (acc == 0)
    {
        return 0.0f;            /* 0 = 不用曲线加减速，按设定速度直接起停 */
    }
    return 20000.0f / (256.0f - (float)acc);
}

/* 预估运动时间。
 * 梯形曲线时 t = N/pps + pps/a_pps 是精确解；
 * 三角形（走不到巡航速度）时会略微高估，对超时判定是安全方向。 */
static uint32_t Chassis_Pos_EstimateMs(int32_t n, uint16_t rpm, uint8_t acc)
{
    float nn  = (float)((n < 0) ? -n : n);
    float pps = (float)rpm * chassis.info.pulsesPerRev / 60.0f;
    float a_rpm_s = Zdt_AccToRpmPerSec(acc);
    float a_pps   = a_rpm_s * chassis.info.pulsesPerRev / 60.0f;
    float t;

    if (pps < 1.0f)
    {
        return 60000u;
    }

    if (a_pps < 1.0f)           /* acc = 0，恒速段 + 起停瞬间完成 */
    {
        t = nn / pps;
    }
    else
    {
        t = nn / pps + pps / a_pps;
    }

    return (uint32_t)(t * 1000.0f);
}

uint8_t Chassis_Move_Distance(float meters, uint16_t rpm, uint8_t acc,
                              uint32_t extra_timeout_ms)
{
    int32_t pulse[CHASSIS_WHEEL_NUM];
    float   mm;

    if (meters == 0.0f)
    {
        return 1;
    }
    if (rpm == 0 || rpm > 3000)
    {
        rpm = CHASSIS_MOVE_RPM;
    }

    mm = meters * 1000.0f;

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float p = mm * chassis.calib.pulsesPerMM[i];

        /* 沿用 Chassis_Motor_Speed_Calc 的符号约定：左前/左后为正，右前/右后为负 */
        if (i == 1 || i == 3)
        {
            p = -p;
        }
        pulse[i] = (int32_t)(p + ((p >= 0.0f) ? 0.5f : -0.5f));

        chassis.pos.cmdPulse[i]      = pulse[i];
        chassis.motor[i].targetSpeed = 0.0f;
    }

    chassis.pos.active   = 1;
    chassis.pos.arrived  = 0;
    chassis.pos.timedout = 0;
    chassis.pos.error    = 0;
    chassis.pos.rpm      = rpm;
    chassis.pos.acc      = acc;
    zdt_arrived_mask     = 0;

    /* 关键：等 MotorTask 正在 DMA 发送的速度帧落地，
     * 否则速度帧会和位置帧抢总线，而且速度帧本身会打断位置运动（手册 5.1.3） */
    osDelay(10);

    if (!set_chassis_pos(&huart1, pulse, rpm, acc))
    {
        chassis.pos.active = 0;
        chassis.pos.error  = 1;
        return 1;                   /* 帧没发全，已撤销缓存，没触发运动 */
    }

    chassis.pos.tStart    = HAL_GetTick();
    chassis.pos.timeoutMs = (uint32_t)((float)Chassis_Pos_EstimateMs(pulse[0], rpm, acc)
                                       * 1.3f) + extra_timeout_ms;

    return 0;
}

uint8_t Chassis_Pos_IsArrived(void)
{
    if (!chassis.pos.active)
    {
        return 1;
    }

    /* 四台都报到位 */
    if (zdt_arrived_mask == 0x0F)
    {
        chassis.pos.active  = 0;
        chassis.pos.arrived = 1;
        return 1;
    }

    /* 兜底：没在电机里开「控制命令应答」就收不到 9F，
     * 这时按预估时间判定走完，属于正常路径，不算错误 */
    if ((uint32_t)(HAL_GetTick() - chassis.pos.tStart) > chassis.pos.timeoutMs)
    {
        chassis.pos.active   = 0;
        chassis.pos.arrived  = 1;
        chassis.pos.timedout = 1;

        /* 有轮子报了到位、有轮子没报 → 才说明真出了问题 */
        if (zdt_arrived_mask != 0x00)
        {
            chassis.pos.error = 1;
        }
        return 1;
    }

    return 0;
}

uint8_t Chassis_Move_Distance_Wait(float meters, uint16_t rpm, uint8_t acc)
{
    if (Chassis_Move_Distance(meters, rpm, acc, 300) != 0)
    {
        return 0;
    }

    while (!Chassis_Pos_IsArrived())
    {
        osDelay(5);
    }

    return (chassis.pos.error == 0) ? 1 : 0;
}

void Chassis_Stop(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis.motor[i].targetSpeed = 0.0f;
    }
    chassis.move.vx = chassis.move.vy = chassis.move.vw = 0.0f;
    chassis.pos.active = 0;

    /* 速度环 4ms 轮完一圈，等它把 0 下发完 */
    osDelay(20);
}
