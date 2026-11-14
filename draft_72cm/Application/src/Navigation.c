#include "Chassis.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include <stdint.h>

/* ============================================================
 * 用法说明
 * ------------------------------------------------------------
 * NAV_MODE_CALIB = 1：标定模式。走 1000mm，方便用卷尺量，
 *                    根据实测距离改写 calib.pulsesPerMM[]。
 * NAV_MODE_CALIB = 0：任务模式。走 720mm。
 * ============================================================ */

#define NAV_MODE_CALIB      1

#define NAV_DISTANCE_M      0.72f
#define NAV_CALIB_M         1.00f

void OS_NavigationCallback(void *argument)
{
    Chassis_Init();                       /* 必须先初始化，否则参数全 0 */

#if NAV_MODE_CALIB
    /* ---- 标定：跑 5 次，尺子量起点到停下后车体参考点的距离 ---- */
    for (uint8_t i = 0; i < 5; i++)
    {
        Chassis_Move_Distance_Wait(NAV_CALIB_M, CHASSIS_MOVE_RPM, CHASSIS_MOVE_ACC);
        osDelay(3000);                    /* 留时间量距离 / 复位 */
    }
#else
    /* ---- 任务：前进 720mm ---- */
    Chassis_Move_Distance_Wait(NAV_DISTANCE_M, CHASSIS_MOVE_RPM, CHASSIS_MOVE_ACC);
#endif

    for (;;)
    {
        osDelay(1000);
    }
}
