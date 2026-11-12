// #ifndef __BSP_USART_H
// #define __BSP_USART_H

// #include "main.h"

// void Bsp_Usart_Init(void);
// void set_motor_speed(UART_HandleTypeDef *huart, uint8_t id, int16_t speed);

// #endif


#ifndef __BSP_UART_H//后期改DMA
#define __BSP_UART_H

#include "main.h"
#include <stdint.h>
#include <math.h>

#define MOTOR_ACC_DEFAULT   0  
#define MOTOR_CHECKSUM      0x6B  

#define YAW_MOTOR_ADDR      0x05 
#define LIFT_MOTOR_ADDR     0x06
#define LIFT_MM_PER_REV     40.0f//一圈移动mm

extern uint8_t usart1RxBuf[25];
extern uint8_t usart2RxBuf[25];
extern uint8_t usart3RxBuf[64];

void set_single_motor_speed(UART_HandleTypeDef *huart, uint8_t addr, float rpm);
void set_chassis_ctrl(UART_HandleTypeDef *huart, float v1, float v2, float v3, float v4);
void set_single_motor_pos(UART_HandleTypeDef *huart, uint8_t addr, float target_pulse, uint16_t rpm, uint8_t acc, uint8_t abs_flag);
void set_gimbal_ctrl(UART_HandleTypeDef *huart, float yaw_pulse, float lift_pulse);

#endif 
