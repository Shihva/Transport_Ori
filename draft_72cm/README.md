# 前进 72cm — 代码草案

**这是草案，没有动 `Transport_car/` 里的任何文件。** 确认没问题后按下面的「迁移步骤」覆盖。

## 文件对应关系

| 草案文件 | 目标位置 | 性质 |
|---|---|---|
| `Bsp/Include/bsp_usart.h` | `Bsp/Include/bsp_usart.h` | 改写 |
| `Bsp/Source/bsp_usart.c` | `Bsp/Source/bsp_usart.c` | 改写 |
| `Application/inc/Chassis.h` | `Application/inc/Chassis.h` | 改写 |
| `Application/src/Chassis.c` | `Application/src/Chassis.c` | 改写 |
| `Application/src/Moto.c` | `Application/src/Moto.c` | 小改（加 4 行） |
| `Application/src/Navigation.c` | `Application/src/Navigation.c` | 改写 |

`Moto.h` 不用动。`Vision.c` / `Gimbal.c` / 所有 CubeMX 生成的文件都不用动。

## 三处核心改动

1. **`bsp_usart.c`：位置帧改成阻塞发送**
   原来 `set_single_motor_pos` 用 `static` 缓冲 + DMA，连发 4 帧时上一帧还没发完就会被覆盖。现在用局部缓冲 + `HAL_UART_Transmit`（13 字节 @115200 约 1.13ms），一次性动作完全可接受。速度帧仍走 DMA，不拖速度环。

2. **`Moto.c`：位置模式期间屏蔽速度帧**
   这是最关键的一行。手册 5.1.3 明确写了位置模式命令可以被新命令打断——现在速度环每 1ms 发一帧，不屏蔽的话位置运动活不过 1ms。

3. **`Chassis.c`：新增定距接口**
   `Chassis_Move_Distance_Wait(0.72f, 150, 176)` 一条语句走完 72cm。

## 发送的帧（默认 150 RPM / acc=176 / 12223 脉冲）

```
01 FD 01 00 96 B0 00 00 2F BF 02 01 6B     ← 左前 addr1 正转
02 FD 00 00 96 B0 00 00 2F BF 02 01 6B     ← 右前 addr2 反转
03 FD 01 00 96 B0 00 00 2F BF 02 01 6B     ← 左后 addr3 正转
04 FD 00 00 96 B0 00 00 2F BF 02 01 6B     ← 右后 addr4 反转
00 FF 66 6B                                 ← 广播触发，四轮同一时刻启动
```

- `02` = 相对当前实时位置（每段都从当前位置起算，不会累积误差）
- `01` = 同步标志「先缓存」，等广播帧才动
- 帧间 3ms 间隔，防粘包

用串口助手可以单独验证：把上面 5 行依次发出去，单独给电机供电就能看到四轮同时转。

标定模式走 1000mm 时，脉冲是 16977（0x4251），帧变成：

```
01 FD 01 00 96 B0 00 00 42 51 02 01 6B
02 FD 00 00 96 B0 00 00 42 51 02 01 6B
03 FD 01 00 96 B0 00 00 42 51 02 01 6B
04 FD 00 00 96 B0 00 00 42 51 02 01 6B
00 FF 66 6B
```

## 参数换算

```
脉冲数      N = 距离(mm) × pulsesPerMM
理论值      3200 / (2π×0.03×1000) = 16.9765 脉冲/mm   (r=30mm 时)
72cm        N = 720 × 16.9765 = 12223
加速度档位  a[rpm/s] = 20000 / (256 − acc)   →  acc = 256 − 20000/a
```

注意 `pulsesPerMM` 是**每毫米**，不是每米。`wheelradius` 单位是米，公式里要 ×1000 换成毫米——这里错一个数量级就会冲出 720 米。

| 巡航 | 线速度 | acc档 | 实际加速度 | 72cm 用时 |
|---|---|---|---|---|
| 100 RPM | 0.314 m/s | 156 | 200 rpm/s | 2.79 s |
| **150 RPM** | **0.471 m/s** | **176** | **250 rpm/s** | **2.13 s** |
| 200 RPM | 0.628 m/s | 206 | 400 rpm/s | 1.65 s |
| 300 RPM | 0.942 m/s | 223 | 600 rpm/s | 1.26 s |

`acc = 0` 是「不使用曲线加减速」，直接按设定速度硬起硬停——**这是原来最大的误差来源，不要用**。

## 标定流程

`Navigation.c` 里把 `NAV_MODE_CALIB` 设成 1，跑 5 次 1000mm，填这张表：

| 轮子 | 地址 | 5 次实测(mm) | 均值 D | K = 16977 / D |
|---|---|---|---|---|
| 左前 | 1 | | | |
| 右前 | 2 | | | |
| 左后 | 3 | | | |
| 右后 | 4 | | | |

然后把 `Chassis_Init()` 里 `calib.pulsesPerMM[]` 的初值换成实测 K，或者运行时调 `Chassis_SetWheelCalib(idx, K)`。

**判断标准：**
- 5 次离散 σ < 3mm → 稳定，微调 K 就能收敛
- σ > 8mm → 打滑或机械问题，先降加速度（acc 调到 150 左右）再标

## 需要注意的坑

1. **心跳保护**：如果电机里开了心跳，位置运动期间 2 秒没收到命令会急停。长距离运动时要么关掉，要么在 `Chassis_Pos_IsArrived()` 里周期性发 `3A` 查询（顺带也能判到位）。
2. **到位判定**：默认靠时间超时结束（估算 ×1.3 + 300ms），不依赖电机回复，`pos.timedout` 会置 1。想精确判到位，在电机菜单里把「控制命令应答」设成 `Reached` 或 `Both`，代码已经会解析 `FD 9F`。只有「部分轮子报到位、部分没报」才算异常（`pos.error=1`）。
3. **模式优先级**：位置模式结束后 `pos.active` 归零，速度环重新接管。如果此时 `chassis.move.vy` 还是非 0（比如 Vision 在写），车会继续走。任务里用完记得 `Chassis_Stop()`。
4. **USART1 接收（重要）**：`stm32f4xx_it.c` 的 `USART1_IRQHandler` 里只有「重新 arm」，**上电后从来没有人第一次 arm**，所以现在整个 USART1 一个字节都收不到。草案用 `Bsp_Usart1_StartRx()` 补上了，由 `Chassis_Init()` 调用。缓冲区 25 字节够用。
5. **链接报错**：如果出现 `undefined reference to sqrtf`，说明没链 libm——本草案已经避开 `sqrtf`/`lroundf`，理论上不会触发。
6. **符号约定**：左前/左后为正，右前/右后为负，和 `Chassis_Motor_Speed_Calc()` 一致。改符号前先用串口助手单独试一台。

## 后续接光流

位置模式是增量式的（`02` 相对当前位置），光流修正天然适配：

```
走 680mm（位置模式，前馈）
→ 光流给出剩余 Δx
→ 再发一条 02 短帧补差额
```

`Chassis_Move_Distance()` 支持重复调用，每次都是「从当前位置再走多少」，不累积误差。轮式里程计可以用 `32`（读取输入脉冲数，累计值）做，注意 `36`（实时位置）在 Emm 固件下只返回单圈 0~65535，不能累计。
