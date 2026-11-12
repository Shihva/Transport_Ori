#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint8_t header;//0xA8

    float INS_yaw;
    float vy_real;
    float vx_real;
    float vw_real;

    float yawAngle;
    float liftHeight;

    uint8_t is_placing;//1为正在放置0为无动作

    float pos_y;//启动时车头前方
    float pos_x;//启动时车左方

    uint8_t end_frame;//0x8A
}VisionTransmit;

typedef struct
{
    uint8_t header;//0xA5

    uint8_t chassis_control;//0stm32控制1上位机控制

    float linear_y;//前正
    float linear_x;//左正
    float angular_z;//逆时针正(rad/s)

    float liftHeight;//mm
    float yawAngle;//度
    // float delta_liftHeight;//mm
    // floar delta_yawAngle;//度

    uint8_t is_grasp;//0张开1闭合

    uint8_t is_light;//0关补光灯1开补光灯

	uint8_t end_frame;//0x5A 
}VisionReceive;

typedef struct
{

}Vision;

void Vision_Receive_DataUpdate(void);

#endif
