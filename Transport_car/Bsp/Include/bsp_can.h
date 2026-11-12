#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "main.h"

#define PULSE_PER_REV 51200
#define INTERVAL_MS 1

#define ANGLE_RAW_TO_RAD 0.00076699039f//2*PI / 8192
#define SPEED_RPM_TO_RAD_S 0.10471975512f//2*PI / 60
#define CURRENT_TO_A 0.001f//mA -> A 

typedef struct {
    int32_t  total_pulse;      // 累计绝对脉冲数
    float    total_angle;      // 累计绝对角度
    float    single_angle;     // 单圈归一化角度
    float    speed_rpm;        // 实时转速 (RPM)
    uint32_t last_update_time; // 上次接收到报文的时间戳 (ms)
} ZDT_Motor_Info_t;

typedef enum {
    MOTOR_MODE_SPEED = 0, //速度模式
    MOTOR_MODE_POS_ABS,   //绝对位置模式
    MOTOR_MODE_POS_REL    //相对位置模式
} motor_mode_e;

void can_user_init(CAN_HandleTypeDef* hcan);
void set_can_ctrl(CAN_HandleTypeDef* hcan,uint32_t id, uint8_t tx_data[], uint8_t dlc);
#endif
