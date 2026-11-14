#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Vision.h"
#include <stdint.h>
#include <math.h>

uint8_t usart1RxBuf[25];
uint8_t usart2RxBuf[25];
uint8_t usart3RxBuf[64];

/* ============================ 发送 ============================ */

/* 等 UART 发送通道空闲，避免 DMA 在途时被下一次调用顶掉 */
static uint8_t uart_wait_idle(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (huart->gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            return 0;
        }
    }
    return 1;
}

/* 速度模式：[addr][0xF6][dir][vH][vL][acc][sync][0x6B]
 * 这是 1ms 周期发送的帧，必须用 DMA，不能阻塞速度环 */
void set_single_motor_speed(UART_HandleTypeDef *huart, uint8_t addr, float rpm)
{
    static uint8_t cmd_speed[8];

    uint16_t vel = (uint16_t)(fabsf(rpm) + 0.5f);
    if (vel > 3000) vel = 3000;

    cmd_speed[0] = addr;
    cmd_speed[1] = 0xF6;
    cmd_speed[2] = (rpm >= 0.0f) ? 0x01 : 0x00;
    cmd_speed[3] = (uint8_t)(vel >> 8);
    cmd_speed[4] = (uint8_t)(vel & 0xFF);
    cmd_speed[5] = MOTOR_ACC_DEFAULT;
    cmd_speed[6] = 0x00;
    cmd_speed[7] = MOTOR_CHECKSUM;

    if (!uart_wait_idle(huart, 2))
    {
        return;                      /* 上一帧还在发，丢掉这一帧，下一周期再补 */
    }
    HAL_UART_Transmit_DMA(huart, cmd_speed, sizeof(cmd_speed));
}

void set_chassis_ctrl(UART_HandleTypeDef *huart, float v1, float v2, float v3, float v4)
{
    static uint8_t step = 0;
    float v_arr[4] = {v1, v2, v3, v4};

    set_single_motor_speed(huart, step + 1, v_arr[step]);
    step = (step + 1) & 0x03;
}

/* 位置模式（Emm 固件）：
 * [addr][0xFD][dir][vH][vL][acc][p31 p24][p23 p16][p15 p8][p7 p0][move_mode][sync][0x6B]
 * 用阻塞发送：帧长 13 字节 @115200 约 1.13ms，一次性动作可接受，
 * 且局部缓冲区天然避免 DMA 缓冲被覆盖的问题 */
uint8_t set_single_motor_pos(UART_HandleTypeDef *huart, uint8_t addr, float target_pulse,
                             uint16_t rpm, uint8_t acc, uint8_t move_mode, uint8_t sync)
{
    uint8_t  buf[13];
    uint32_t pulses = (uint32_t)(fabsf(target_pulse) + 0.5f);

    buf[0]  = addr;
    buf[1]  = 0xFD;
    buf[2]  = (target_pulse >= 0.0f) ? 0x01 : 0x00;
    buf[3]  = (uint8_t)(rpm >> 8);
    buf[4]  = (uint8_t)(rpm & 0xFF);
    buf[5]  = acc;
    buf[6]  = (uint8_t)(pulses >> 24);
    buf[7]  = (uint8_t)(pulses >> 16);
    buf[8]  = (uint8_t)(pulses >> 8);
    buf[9]  = (uint8_t)(pulses & 0xFF);
    buf[10] = move_mode;
    buf[11] = sync;
    buf[12] = MOTOR_CHECKSUM;

    if (!uart_wait_idle(huart, 20))
    {
        return 0;
    }
    return (HAL_UART_Transmit(huart, buf, sizeof(buf), 20) == HAL_OK) ? 1 : 0;
}

/* 广播触发多机同步运动 */
void set_multi_sync_trigger(UART_HandleTypeDef *huart)
{
    uint8_t buf[4] = {0x00, 0xFF, 0x66, MOTOR_CHECKSUM};

    if (!uart_wait_idle(huart, 20))
    {
        return;
    }
    HAL_UART_Transmit(huart, buf, sizeof(buf), 20);
}

/* 4 条位置帧（sync=1 只缓存）→ 广播触发。
 * 帧间 3ms 间隔，手册 3.5 明确要求防粘包。
 *
 * 安全处理：只要有一帧没发出去就绝不能广播——三台动一台不动会直接让车歪着冲出去。
 * 此时用 0 脉冲覆盖缓存，让后续任何广播都变成原地不动。 */
uint8_t set_chassis_pos(UART_HandleTypeDef *huart, const int32_t pulse[4],
                        uint16_t rpm, uint8_t acc)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!set_single_motor_pos(huart, (uint8_t)(i + 1), (float)pulse[i],
                                  rpm, acc, ZDT_MOVE_REL_CURRENT, ZDT_SYNC_CACHE))
        {
            for (uint8_t k = 0; k < 4; k++)
            {
                (void)set_single_motor_pos(huart, (uint8_t)(k + 1), 0.0f,
                                           rpm, acc, ZDT_MOVE_REL_CURRENT, ZDT_SYNC_CACHE);
                osDelay(3);
            }
            return 0;
        }
        osDelay(3);
    }

    set_multi_sync_trigger(huart);
    return 1;
}

void set_gimbal_ctrl(UART_HandleTypeDef *huart, float yaw_pulse, float lift_pulse)
{
    static uint8_t axis_step = 0;

    if (axis_step == 0)
    {
        (void)set_single_motor_pos(huart, YAW_MOTOR_ADDR, yaw_pulse, 150, 200,
                                   ZDT_MOVE_ABS, ZDT_SYNC_NOW);
        axis_step = 1;
    }
    else
    {
        (void)set_single_motor_pos(huart, LIFT_MOTOR_ADDR, lift_pulse, 300, 200,
                                   ZDT_MOVE_ABS, ZDT_SYNC_NOW);
        axis_step = 0;
    }
}

/* ============================ 接收 ============================ */

volatile uint8_t zdt_arrived_mask = 0;   /* bit0..3 = 地址 1..4 到位 */

#define ZDT_RX_FRAME_MAX   16

static uint8_t rx_frame[ZDT_RX_FRAME_MAX];
static uint8_t rx_len = 0;

static void zdt_handle_frame(const uint8_t *f, uint8_t n)
{
    uint8_t addr = f[0];
    uint8_t bit;

    if (addr < 1 || addr > 4)
    {
        return;
    }
    bit = (uint8_t)(1u << (addr - 1));

    /* 命令确认：Addr Func 02 6B */
    if (n == 4 && f[2] == 0x02)
    {
        return;
    }

    /* 到位返回：Addr FD 9F 6B（需在电机里把「控制命令应答」设为 Reached/Both） */
    if (n == 4 && f[1] == 0xFD && f[2] == 0x9F)
    {
        zdt_arrived_mask |= bit;
        return;
    }

    /* 状态标志：Addr 3A Status 6B，bit1 = Prf_TF 位置到达 */
    if (n == 4 && f[1] == 0x3A)
    {
        if (f[2] & 0x02)
        {
            zdt_arrived_mask |= bit;
        }
        return;
    }

    /* 累计输入脉冲：Addr 32 sign p31..p0 6B（8 字节），留给后续轮式里程计 */
    /* 需要时在这里累加，注意 Emm 固件返回的是带符号累计值 */
}

/* 逐字节喂状态机。所有响应帧都以 0x6B 结尾。
 * 注意：如果 3A 的状态字节恰好等于 0x6B 会被误判，概率极低且只影响一次轮询 */
void Bsp_Usart1_Parse(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];

        if (rx_len == 0)
        {
            if (b >= 1 && b <= 4)          /* 只接受地址 1..4 作为帧首 */
            {
                rx_frame[rx_len++] = b;
            }
            continue;
        }

        if (rx_len >= ZDT_RX_FRAME_MAX)
        {
            rx_len = 0;
            continue;
        }

        rx_frame[rx_len++] = b;

        if (b == MOTOR_CHECKSUM)
        {
            zdt_handle_frame(rx_frame, rx_len);
            rx_len = 0;
        }
    }
}

/* 首次 arm USART1 的 DMA 空闲接收。
 * stm32f4xx_it.c 的 USART1_IRQHandler 里只做「重新 arm」，
 * 上电后必须有人先收一次，否则第一个字节永远不来 */
void Bsp_Usart1_StartRx(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1RxBuf, sizeof(usart1RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        /* 重新 arm 的动作在 stm32f4xx_it.c 的 USART1_IRQHandler 里，这里只做解析 */
        Bsp_Usart1_Parse(usart1RxBuf, Size);
    }
    else if (huart->Instance == USART2)
    {
        /* 预留 */
    }
    else if (huart->Instance == USART3)
    {
        Vision_Receive_DataUpdate();
    }
}
