
#include "Chassis.h"
#include "cmsis_os.h"

#define CRUISE_V  0.20f
void OS_NavigationCallback(void *argument)
{
    Chassis_Init();                 /* ① 少了这个会 3000 RPM 冲出去 */

    chassis.move.vy = CRUISE_V;     /* ② 开始走 */
    osDelay(6000);                  /* ③ 走 1.5 秒 = 30cm */
    chassis.move.vy = 0.0f;         /* ④ 必须停，否则一直走 */

    for (;;)                        /* ⑤ 必须不返回，否则系统冻结 */
    {
        osDelay(1000);
    }
}
volatile const uint32_t nav_version = 6000;