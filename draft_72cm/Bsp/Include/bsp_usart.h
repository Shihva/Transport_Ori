#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include <stdint.h>
#include <math.h>

#define MOTOR_ACC_DEFAULT   0
#define MOTOR_CHECKSUM      0x6B

#define YAW_MOTOR_ADDR      0x05
#define LIFT_MOTOR_ADDR     0x06
#define LIFT_MM_PER_REV     40.0f

/* ---------- ZDT 位置模式「运动模式」字节（手册 5.3.12） ---------- */
#define ZDT_MOVE_REL_LAST     0x00   /* 相对上一输入目标位置 */
#define ZDT_MOVE_ABS          0x01   /* 相对坐标零点（绝对） */
#define ZDT_MOVE_REL_CURRENT  0x02   /* 相对当前实时位置（推荐） */

/* ---------- ZDT 「同步标志」字节 ---------- */
#define ZDT_SYNC_NOW          0x00   /* 立即执行 */
#define ZDT_SYNC_CACHE        0x01   /* 先缓存，等广播 00 FF 66 6B 触发 */

/* ---------- 底盘位置模式参数 ---------- */
#define CHASSIS_MOVE_RPM      150    /* 巡航转速，建议 100~200 */
#define CHASSIS_MOVE_ACC      176    /* 加速度档位，对应约 250 RPM/s */

extern uint8_t usart1RxBuf[25];
extern uint8_t usart2RxBuf[25];
extern uint8_t usart3RxBuf[64];

/* bit0..bit3 = 地址 1..4 的到位标志，由 USART1 接收解析维护 */
extern volatile uint8_t zdt_arrived_mask;

/* 速度模式（0xF6）：保留原 DMA 发送 */
void set_single_motor_speed(UART_HandleTypeDef *huart, uint8_t addr, float rpm);
void set_chassis_ctrl(UART_HandleTypeDef *huart, float v1, float v2, float v3, float v4);

/* 位置模式（0xFD）：改为阻塞发送，保证多帧连发不丢包
 * 返回 1 = 发送成功，0 = 通道忙/超时（帧没发出去） */
uint8_t set_single_motor_pos(UART_HandleTypeDef *huart, uint8_t addr, float target_pulse,
                             uint16_t rpm, uint8_t acc, uint8_t move_mode, uint8_t sync);

/* 一次发完 4 台位置帧 + 广播触发，四轮严格同时启动
 * 返回 1 = 全部发出并已触发，0 = 中途失败（已撤销缓存，未触发） */
uint8_t set_chassis_pos(UART_HandleTypeDef *huart, const int32_t pulse[4],
                        uint16_t rpm, uint8_t acc);

/* 广播触发多机同步运动：00 FF 66 6B */
void set_multi_sync_trigger(UART_HandleTypeDef *huart);

void set_gimbal_ctrl(UART_HandleTypeDef *huart, float yaw_pulse, float lift_pulse);

/* 接收解析入口，由 HAL_UARTEx_RxEventCallback 调用 */
void Bsp_Usart1_Parse(const uint8_t *data, uint16_t len);

/* 首次启动 USART1 的 DMA 空闲接收。
 * 必须在上电后调用一次——stm32f4xx_it.c 里的重 arm 只在收到中断后才生效，
 * 没人先 arm 的话永远收不到第一个字节。Chassis_Init() 已代为调用。 */
void Bsp_Usart1_StartRx(void);

#endif
